import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class ComputedTorqueController(Node):
    def __init__(self):
        super().__init__('computed_torque_controller')
        
        # Publishers & Subscribers
        self.publisher_ = self.create_publisher(Float64MultiArray, '/torque_controller/commands', 10)
        self.subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # --- NUEVO SUSCRIPTOR PARA MOVER EL ROBOT EN TIEMPO REAL ---
        self.target_sub = self.create_subscription(Float64MultiArray, '/target_position', self.target_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.01, self.control_loop)
        
        # Estado actual del robot
        self.q = [0.0, 0.0]
        self.q_dot = [0.0, 0.0]
        self.tiempo = 0.0
        self.listo = False

        # --- VARIABLES DE OBJETIVO (TARGET) ---
        self.q_d = [math.pi / 2.0, 0.0]  # Posición inicial por defecto (90 grados)
        self.q_d_dot = [0.0, 0.0]
        self.q_d_ddot = [0.0, 0.0]

        self.get_logger().info("Controlador de Par Computado (PD) INICIADO. Esperando comandos en /target_position")

    # Lee la posición real del robot
    def joint_state_callback(self, msg):
        try:
            idx1 = msg.name.index('joint1')
            idx2 = msg.name.index('joint2')
            self.q = [msg.position[idx1], msg.position[idx2]]
            self.q_dot = [msg.velocity[idx1], msg.velocity[idx2]]
            self.listo = True
        except ValueError:
            pass

    # --- NUEVA FUNCIÓN CALLBACK PARA ACTUALIZAR EL OBJETIVO ---
    def target_callback(self, msg):
        if len(msg.data) >= 2:
            self.q_d = [msg.data[0], msg.data[1]]
            self.get_logger().info(f"Nuevo objetivo recibido: q1={self.q_d[0]:.3f}, q2={self.q_d[1]:.3f}")

    def control_loop(self):
        if not self.listo: return 
            
        self.tiempo += 0.01
        
        # Usa las variables de clase (self.q_d) en lugar de valores fijos
        tau = self.calcular_torque(self.q, self.q_dot, self.q_d, self.q_d_dot, self.q_d_ddot)
        
        msg_torque = Float64MultiArray()
        msg_torque.data = tau
        self.publisher_.publish(msg_torque)
        
        # Imprime logs cada 1 segundo
        if int(self.tiempo * 100) % 100 == 0:
            self.get_logger().info(
                f'q: [{self.q[0]:+.3f}, {self.q[1]:+.3f}] | '
                f'err: [{(self.q_d[0]-self.q[0]):+.3f}, {(self.q_d[1]-self.q[1]):+.3f}] | '
                f'tau: [{tau[0]:+.2f}, {tau[1]:+.2f}]'
            )

    def calcular_torque(self, q, q_dot, q_d, q_d_dot, q_d_ddot):
        # Parámetros (Asegúrate de que coincidan con tu URDF)
        m1 = 1.2; m2 = 1.0
        L1 = 0.5; L2 = 0.4
        lc1 = 0.25; lc2 = 0.2
        I1 = 0.05; I2 = 0.04
        g = 9.81
        b1 = 0.1; b2 = 0.1
        
        q1, q2 = q[0], q[1]
        qd1, qd2 = q_dot[0], q_dot[1]

        # Matrices dinámicas
        M11 = m1*lc1**2 + I1 + m2*(L1**2 + lc2**2 + 2*L1*lc2*np.cos(q2)) + I2
        M12 = m2*(lc2**2 + L1*lc2*np.cos(q2)) + I2
        M = np.array([[M11, M12], [M12, m2*lc2**2 + I2]])

        h = m2*L1*lc2*np.sin(q2)
        C = np.array([[-h*qd2, -h*(qd1 + qd2)], [h*qd1, 0.0]])

        G1 = (m1*lc1 + m2*L1)*g*np.cos(q1) + m2*g*lc2*np.cos(q1+q2)
        G2 = m2*g*lc2*np.cos(q1+q2)
        G = np.array([G1, G2])

        Fv = np.array([b1*qd1, b2*qd2])

        # --- CONTROL PD (Sin integral para evitar oscilación) ---
        Kp = np.diag([80.0, 70.0])
        Kd = np.diag([22.0, 20.0])
        
        e = np.array(q_d) - np.array(q)
        e_dot = np.array(q_d_dot) - np.array(q_dot)
        
        # u = ddq_d + Kp*e + Kd*edot
        u = np.array(q_d_ddot) + np.dot(Kp, e) + np.dot(Kd, e_dot)

        # Ley de control: tau = M*u + C*q_dot + G + Fv
        tau = np.dot(M, u) + np.dot(C, np.array(q_dot)) + G + Fv

        return tau.tolist()

def main(args=None):
    rclpy.init(args=args)
    nodo = ComputedTorqueController()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
