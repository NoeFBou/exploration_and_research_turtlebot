import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO 

class SimuAI(Node):
    def __init__(self):
        super().__init__('simu_ai_node')
        
        topic_name = '/oakd/rgb/image_raw'
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        self.model = YOLO("yolov8n.pt") 

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            results = self.model(cv_image, verbose=False)
            
            annotated_frame = results[0].plot()
            cv2.imshow("Vue Robot - Detection IA", annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Erreur: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimuAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()