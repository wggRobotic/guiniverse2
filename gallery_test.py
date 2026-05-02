import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class DummyCompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('dummy_compressed_image_publisher')

        self.publisher = self.create_publisher(
            CompressedImage,
            '/quac/qrcodes/images',
            10
        )

        self.json_publisher = self.create_publisher(
            String,
            '/quac/qrcodes/json',
            10
        )

        # Publish immediately after startup
        self.create_timer(0.5, self.publish_images_once)
        self.published = False

    def publish_images_once(self):
        if self.published:
            return

        self.get_logger().info("Publishing 3 compressed images...")

        for i in range(3):
            # Create a dummy image (different colors for variety)
            img = np.zeros((240, 320, 3), dtype=np.uint8)
            img[:, :] = (i * 80, 255 - i * 80, i * 40)

            # Encode as JPEG using OpenCV (no cv_bridge)
            success, encoded_img = cv2.imencode('.jpg', img)
            if not success:
                self.get_logger().error("Failed to encode image")
                continue

            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"dummy_frame_{i}"
            msg.format = "jpeg"
            msg.data = encoded_img.tobytes()

            self.publisher.publish(msg)
            self.get_logger().info(f"Published image {i+1}")

        self.published = True

        msg = String()
        msg.data = """{
  {
    name: dummy_frame_0,
    type: type_0,
    confidence: 0.67
  },

  {
    name: dummy_frame_1,
    type: type_1,
    confidence: 0.67
  },

  {
    name: dummy_frame_2,
    type: type_2,
    confidence: 0.67
  }
}
"""
        self.json_publisher.publish(msg)

        # Shutdown after publishing
        self.get_logger().info("Done publishing. Shutting down...")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DummyCompressedImagePublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()