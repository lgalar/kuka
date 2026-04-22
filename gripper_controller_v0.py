#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 # 1. Importamos el tipo de mensaje exacto que espera la ESP32 [cite: 248, 554]

class KukaGripperController(Node):

    def __init__(self):
        super().__init__("gripper_controller_node")
        
        # 2. Creamos el publicador apuntando al tópico al que está suscrita la ESP32 
        self.publisher_ = self.create_publisher(Int32, '/esp32_sub', 10)
        
        # Estado inicial de la pinza (empezamos mandando un 0 para asegurar que esté abierta)
        self.estado_pinza = 0 
        
        self.get_logger().info("Nodo controlador de pinza KUKA iniciado.")
        
        # El temporizador llamará a la función cada 3.0 segundos
        self.create_timer(3.0, self.timer_callback)

    def timer_callback(self):
        # 3. Instanciamos el objeto del mensaje y le asignamos nuestro valor (1 o 0)
        msg = Int32()
        msg.data = self.estado_pinza
        
        # 4. Publicamos el mensaje en la red de ROS 2
        self.publisher_.publish(msg)
        
        # Imprimimos en la terminal para confirmar visualmente qué se mandó
        if self.estado_pinza == 1:
            self.get_logger().info(f"Comando enviado: {msg.data} -> (CERRANDO pinza)")
            self.estado_pinza = 0 # Preparamos el 0 para abrir en el siguiente ciclo
        else:
            self.get_logger().info(f"Comando enviado: {msg.data} -> (ABRIENDO pinza)")
            self.estado_pinza = 1 # Preparamos el 1 para cerrar en el siguiente ciclo

def main(args=None):
    rclpy.init(args=args)
    node = KukaGripperController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()