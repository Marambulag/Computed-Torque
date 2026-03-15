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
        self.create_subscription(Imu, '/link1/imu', self.imu1_callback, 10)
        self.create_subscription(Imu, '/link2/imu', self.imu2_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # --- PUBLICADORES PARA GRAFICAR ---
        self.ekf_pub = self.create_publisher(JointState, '/ekf_joint_states', 10)
        self.imu_pub = self.create_publisher(JointState, '/imu_raw_joint_states', 10)

        # Timer
        self.timer = self.create_timer(0.01, self.loop)

        # Initialization flags
        self.initialized = False
        self.imu1_received = False
        self.imu2_received = False

        # Estados (States)
        self.theta_imu = np.array([0.0, 0.0]) # Integración pura
        self.theta_ekf = np.array([0.0, 0.0]) # Salida del EKF
        self.theta_gt = np.array([0.0, 0.0])  # Ground Truth de Gazebo

        # Variables de los sensores
        self.w = np.array([0.0, 0.0])             
        self.accel_angle = np.array([0.0, 0.0])   

        # EKF matrices (P, Q, R)
        self.P = np.eye(2) * 0.1
        self.Q = np.eye(2) * 0.001 
        self.R = np.eye(2) * 0.5   

        self.last_time = self.get_clock().now()
        self.t = 0.0

        self.get_logger().info("IMU State Estimator Iniciado. Comparando Raw vs EKF vs Ground Truth.")

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

        if not self.initialized:
            start_q1 = self.accel_angle[0]
            start_q2 = self.accel_angle[1] - self.accel_angle[0]
            self.theta_ekf = np.array([start_q1, start_q2])
            self.theta_imu = np.array([start_q1, start_q2])
            self.initialized = True
            return

        self.t += dt

        # 1. Integración Pura IMU (IMU RAW)
        self.theta_imu[0] += self.w[0] * dt
        self.theta_imu[1] += (self.w[1] - self.w[0]) * dt

        # 2. EKF PREDICT
        x_pred = np.zeros(2)
        x_pred[0] = self.theta_ekf[0] + self.w[0] * dt
        x_pred[1] = self.theta_ekf[1] + (self.w[1] - self.w[0]) * dt
        P_pred = self.P + self.Q 

        # 3. EKF UPDATE
        z = np.zeros(2)
        z[0] = self.accel_angle[0]
        z[1] = self.accel_angle[1] - self.accel_angle[0]

        H = np.eye(2)
        S = H @ P_pred @ H.T + self.R
        K = P_pred @ H.T @ np.linalg.inv(S)
        y = z - H @ x_pred

        self.theta_ekf = x_pred + K @ y
        self.P = (np.eye(2) - K @ H) @ P_pred

        # --- PUBLICAR PARA GRAFICAR ---
        stamp = self.get_clock().now().to_msg()
        m_ekf = JointState(); m_ekf.header.stamp = stamp
        m_ekf.name = ['j1','j2']; m_ekf.position = self.theta_ekf.tolist()
        self.ekf_pub.publish(m_ekf)

        m_imu = JointState(); m_imu.header.stamp = stamp
        m_imu.name = ['j1','j2']; m_imu.position = self.theta_imu.tolist()
        self.imu_pub.publish(m_imu)

        # --- LOGS EN TERMINAL (Cada 0.5 segundos) ---
        if int(self.t * 100) % 50 == 0:
            self.get_logger().info("-" * 40)
            self.get_logger().info(f"IMU RAW (Deriva): q1={self.theta_imu[0]:+.3f}, q2={self.theta_imu[1]:+.3f}")
            self.get_logger().info(f"EKF EST (Limpio): q1={self.theta_ekf[0]:+.3f}, q2={self.theta_ekf[1]:+.3f}")
            self.get_logger().info(f"GROUND TRUTH:    q1={self.theta_gt[0]:+.3f}, q2={self.theta_gt[1]:+.3f}")

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
