import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
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
            qos_profile_sensor_data)
        
        self.yolo_pub_ = self.create_publisher(String, 'MasterJo_YOLO', qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image,'feed_YOLO', qos_profile_sensor_data)

        self.br = CvBridge()
        self.MIN_BOX_AREA = 3700  

        cv2.namedWindow('Processed Image')
        cv2.namedWindow('YOLO Object Detection') 
    
    def listener_callback(self, data):
        try:
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        results = self.model(current_frame, verbose=False)

        # YOLO 이미지 그리기
        annotated_frame = results[0].plot()

        # YOLO 결과 처리
        if len(results[0].boxes) > 0:
            for box in results[0].boxes:
                x, y, w, h = box.xywh[0].tolist()
                area = w * h
                
                if area > self.MIN_BOX_AREA:
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