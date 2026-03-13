import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState

class ImuStateEstimator(Node):

    def __init__(self):
        super().__init__('imu_state_estimator')

        # Subscribers
        self.create_subscription(Imu, '/link1/imu', self.imu1_callback, 1)
        self.create_subscription(Imu, '/link2/imu', self.imu2_callback, 1)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 1) # Solo para comparar

        # --- NUEVO: PUBLICADOR DEL EKF ---
        self.ekf_pub = self.create_publisher(JointState, '/ekf_joint_states', 1)

        # Timer
        self.timer = self.create_timer(0.01, self.loop)

        # Initialization flags
        self.initialized = False
        self.imu1_received = False
        self.imu2_received = False

        # Estados (States)
        self.theta_imu = np.array([0.0, 0.0]) # Pure integration
        self.theta_ekf = np.array([0.0, 0.0]) # EKF output
        self.theta_gt = np.array([0.0, 0.0])  # Ground truth from joint_states

        # Variables de los sensores
        self.w = np.array([0.0, 0.0])             # Velocidades angulares (Gyro)
        self.accel_angle = np.array([0.0, 0.0])   # Ángulos absolutos (Accel)

        # EKF matrices (P, Q, R)
        self.P = np.eye(2) * 0.1
        self.Q = np.eye(2) * 0.001 
        self.R = np.eye(2) * 0.5   

        self.last_time = self.get_clock().now()
        self.t = 0.0

        self.get_logger().info("IMU State Estimator Iniciado. Esperando datos de sensores...")

    def imu1_callback(self, msg):
        self.w[0] = msg.angular_velocity.y
        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z
        self.accel_angle[0] = math.atan2(ax, az)
        self.imu1_received = True

    def imu2_callback(self, msg):
        self.w[1] = msg.angular_velocity.y
        ax = msg.linear_acceleration.x
        az = msg.linear_acceleration.z
        self.accel_angle[1] = math.atan2(ax, az)
        self.imu2_received = True

    def joint_callback(self, msg):
        try:
            i1 = msg.name.index('joint1')
            i2 = msg.name.index('joint2')
            self.theta_gt[0] = msg.position[i1]
            self.theta_gt[1] = msg.position[i2]
        except ValueError:
            pass

    def loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0 or not (self.imu1_received and self.imu2_received):
            return

        # =========================
        # INITIALIZATION BLOCK
        # =========================
        if not self.initialized:
            start_q1 = self.accel_angle[0]
            start_q2 = self.accel_angle[1] - self.accel_angle[0]
            
            self.theta_ekf = np.array([start_q1, start_q2])
            self.theta_imu = np.array([start_q1, start_q2])
            
            self.initialized = True
            self.get_logger().info(f"Filtro Inicializado en: q1={start_q1:.3f}, q2={start_q2:.3f}")
            return

        self.t += dt

        # IMU PURE INTEGRATION
        self.theta_imu[0] += self.w[0] * dt
        self.theta_imu[1] += (self.w[1] - self.w[0]) * dt

        # =========================
        # 1. EKF PREDICT (Gyro)
        # =========================
        x_pred = np.zeros(2)
        x_pred[0] = self.theta_ekf[0] + self.w[0] * dt
        x_pred[1] = self.theta_ekf[1] + (self.w[1] - self.w[0]) * dt

        P_pred = self.P + self.Q 

        # =========================
        # 2. EKF UPDATE (Accelerometer)
        # =========================
        z = np.zeros(2)
        z[0] = self.accel_angle[0]
        z[1] = self.accel_angle[1] - self.accel_angle[0]

        H = np.eye(2)
        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T @ np.linalg.inv(S)
        y = z - H @ x_pred

        self.theta_ekf = x_pred + K @ y
        self.P = (np.eye(2) - K @ H) @ P_pred

        # =========================
        # 3. PUBLICAR AL CONTROLADOR
        # =========================
        msg_ekf = JointState()
        msg_ekf.header.stamp = self.get_clock().now().to_msg()
        msg_ekf.name = ['joint1', 'joint2']
        msg_ekf.position = [float(self.theta_ekf[0]), float(self.theta_ekf[1])]
        msg_ekf.velocity = [float(self.w[0]), float(self.w[1] - self.w[0])]
        self.ekf_pub.publish(msg_ekf)

        # =========================
        # LOGS
        # =========================
        if int(self.t * 100) % 50 == 0:
            self.get_logger().info(
                f"EKF_EST (Stable)   q1={self.theta_ekf[0]:+.3f}  q2={self.theta_ekf[1]:+.3f} | GT q1={self.theta_gt[0]:+.3f} q2={self.theta_gt[1]:+.3f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ImuStateEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
