#!/usr/bin/env python3
import rclpy
import threading
from sensor_msgs.msg import JointState

def main():
 rclpy.init()
 # Declarar el nombre del nodo
 node = rclpy.create_node('JointsNode')
 
 # Definir el publicador
 
 
 thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
 thread.start()
 
 # Definir una variable de tipo JointState
 
 
 # Definir las propiedades de la variable JointState
 # - Definir el tiempo en el stamp

 # - Definir los nombres de la articulacion como una lista

 # - Definir las posiciones articulares como una lista
 
 # Definir un bucle y su tiempo de muestreo, dentro del bucle, publicar el mensaje
 
 
 
 
if __name__ == '__main__':
 main()
