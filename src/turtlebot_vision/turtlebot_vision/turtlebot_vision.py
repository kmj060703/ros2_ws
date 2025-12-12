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
        
        self.br = CvBridge()
        cv2.namedWindow('Processed Image')
        cv2.namedWindow('YOLO Object Detection') 
    
    def listener_callback(self, data):
        current_frame = self.br.imgmsg_to_cv2(data)
        
        results = self.model(current_frame, verbose=False)
        annotated_frame = results[0].plot() 

        if len(results[0].boxes) > 0:
            box = results[0].boxes[0]
            class_id = int(box.cls[0].item())
            confidence = box.conf[0].item()
            object_name = results[0].names[class_id]

            msg = String()
            msg.data = f"{object_name},{confidence:.2f}"

            self.yolo_pub_.publish(msg)

        cv2.imshow("Processed Image", current_frame)
        cv2.moveWindow('Processed Image', 0, 300)
        cv2.imshow('YOLO Object Detection', annotated_frame)  
        cv2.moveWindow('YOLO Object Detection', 0, 1000)
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