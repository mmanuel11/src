#!/usr/bin/env python3
import rclpy
import time
import serial
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from proyecto_interfaces.msg import String

ser = serial.Serial("/dev/ttyACM0", baudrate=115200)


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('connection_serial')


        self.arduino = self.create_subscription(
            String,
            '/arduinoSerial',
            self.listener_callback,
            8)

    def listener_callback(self, msg):
             
        comando = msg.data + "\n"
        comandoBytes = comando.encode()
        ser.write(comandoBytes)
        print(ser.readline())
        self.get_logger().info(comando)
       


            
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber= MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    ser.close()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()