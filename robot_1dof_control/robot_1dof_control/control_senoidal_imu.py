import rclpy
from rclpy.node import Node
import math

# Importamos los mensajes estándar de ROS 2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu

class ControlPenduloIMU(Node):
    def __init__(self):
        super().__init__('control_pendulo_imu')
        
        # 1. PUBLICADOR: Envía la señal de torque al controlador
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/torque_controller/commands', 
            10
        )
        
        # 2. SUSCRIPTOR: Escucha la IMU desde Gazebo
        self.subscriber_ = self.create_subscription(
            Imu,
            '/link1/ruido_imu',
            self.imu_callback,
            10
        )
        
        # 3. TEMPORIZADOR: Ejecuta el lazo de control a 100 Hz (cada 0.01 segundos)
        self.timer = self.create_timer(0.01, self.lazo_de_control)
        
        # Variables para calcular el tiempo y guardar la lectura del sensor
        self.tiempo = 0.0
        self.velocidad_angular_z = 0.0

        self.get_logger().info("Nodo iniciado. Inyectando torque senoidal al péndulo...")

    def imu_callback(self, msg):
        # Guardamos la velocidad angular en el eje Z (el eje de rotación de nuestro joint)
        self.velocidad_angular_z = msg.angular_velocity.z

    def lazo_de_control(self):
        self.tiempo += 0.01
        
        # --- CAMBIO PARA LA PRUEBA ---
        # La amplitud ahora crece 0.1 Nm por cada segundo de simulación
        amplitud_dinamica = 1.0 + (0.1 * self.tiempo) 
        frecuencia = 2.0 
        
        tau = amplitud_dinamica * math.sin(frecuencia * self.tiempo)
        # -----------------------------
        
        msg_torque = Float64MultiArray()
        msg_torque.data = [tau]
        self.publisher_.publish(msg_torque)
        
        self.get_logger().info(
            f'Amplitud: {amplitud_dinamica:.2f} | Torque: {tau:+.2f} Nm | Vel Z: {self.velocidad_angular_z:+.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    nodo = ControlPenduloIMU()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        nodo.get_logger().info("Nodo detenido manualmente.")
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()