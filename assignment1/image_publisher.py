#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.image_files = glob.glob('images/*.jpg')  # folder with your images
        self.index = 0
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2 fps

    def timer_callback(self):
        if not self.image_files:
            self.get_logger().warning("No images found in 'images/' folder")
            return

        img = cv2.imread(self.image_files[self.index])
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info(f'Published {self.image_files[self.index]}')

        self.index = (self.index + 1) % len(self.image_files)

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
