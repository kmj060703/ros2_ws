import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

# pip uninstall -y opencv-python opencv-contrib-python numpy
# pip install numpy==1.23.5
# pip install opencv-python==4.6.0.66
# pip install opencv-python==4.6.0.66

# run : 그냥 vscode에서 
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('turtlebot_vision_subscriber')
        
        # YOLO 모델
        self.model = YOLO('/home/bws/Downloads/best.pt')
        
        self.subscription = self.create_subscription(
            Image,
            '/vision/image_processed',
            self.listener_callback,
            10)
        self.subscription  
        self.br = CvBridge()
        cv2.namedWindow('Processed Image')
        cv2.namedWindow('YOLO Object Detection') 
    
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        
        # YOLO 추론
        results = self.model(current_frame)
        annotated_frame = results[0].plot() 
        
        # 이미지 표시
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