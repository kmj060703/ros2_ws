import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('turtlebot_vision_subscriber')
        
        self.model = YOLO('src/best.pt')
        
        self.subscription = self.create_subscription(
            Image,
            '/vision/image_processed',
            self.listener_callback,
            10)
        
        self.yolo_pub_ = self.create_publisher(String, 'MasterJo_YOLO', 10)
        self.image_pub = self.create_publisher(Image,'feed_YOLO',10)

        self.br = CvBridge()
        cv2.namedWindow('Processed Image')
        cv2.namedWindow('YOLO Object Detection') 
    
    def listener_callback(self, data):
      current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

      results = self.model(current_frame, verbose=False)

    #  YOLO 이미지
      annotated_frame = results[0].plot()

    # YOLO 결과 
      if len(results[0].boxes) > 0:
        box = results[0].boxes[0]
        class_id = int(box.cls[0])
        confidence = float(box.conf[0])
        object_name = results[0].names[class_id]

        msg = String()
        msg.data = f"{object_name},{confidence:.2f}"
        self.yolo_pub_.publish(msg)

   
      img_msg = self.br.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
      img_msg.header = data.header
      self.image_pub.publish(img_msg)

   
      cv2.imshow("Processed Image", current_frame)
      cv2.imshow("YOLO Object Detection", annotated_frame)
      cv2.waitKey(1)
 

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()