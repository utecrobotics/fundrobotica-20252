import rclpy
from std_msgs.msg import Int32
import time

def main():
  rclpy.init()

  node = rclpy.create_node('publicador')
  publisher = node.create_publisher(Int32, 'counter', 10)

  msg = Int32()
  
  node.declare_parameter("count", 0)
  count = node.get_parameter("count").get_parameter_value().integer_value
  
  try:
    while rclpy.ok():
      msg.data = count
      publisher.publish(msg)
      node.get_logger().info(f'Publishing: {msg.data}')
      count += 1
      time.sleep(0.5)
  except KeyboardInterrupt:
    node.get_logger().info('Publisher stopped by user')
  
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
