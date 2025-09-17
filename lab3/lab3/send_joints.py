#!/usr/bin/env python3
import rclpy
from std_msgs.msg import Float64MultiArray

def main():

  rclpy.init()
  # Declarar el nombre del nodo
  node = rclpy.create_node('JointsNode')
 
  # Definir el publicador para los topicos: /forward_position_controller/commands
  # Se puede revisar el tipo de mensaje que usa el topico, con el comando: $ ros2 topic info <nombre_topico>
 
 
  # Definir una variable de tipo
 
 
  # Definir las propiedades de la variable
 
 
  # Definir la logica del codigo para que se mueva de una posicion a otra
  
  
  
if __name__ == "__main__":
  main()
