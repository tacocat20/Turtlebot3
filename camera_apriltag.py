# camera_apriltag.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import apriltag

class CameraAprilTagNode(Node):
    def __init__(self):
        super().__init__('camera_apriltag')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30 FPS
        self.detector = apriltag.Detector()
        self.max_turn_rate = 0.5  # adjustable angular velocity
        self.forward_speed = 0.1  # adjustable linear velocity

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # flip if your webcam is upside down
        frame = cv2.flip(frame, -1)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        twist = Twist()

        if results:
            # take the first tag detected
            tag = results[0]
            cx, cy = tag.center
            frame_width = frame.shape[1]

            # compute angular velocity to steer toward tag
            error = cx - frame_width / 2
            twist.linear.x = self.forward_speed
            twist.angular.z = -error / (frame_width / 2) * self.max_turn_rate

            # optional: draw rectangle and tag ID
            (x, y, w, h) = tag.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, str(tag.tag_id), (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.pub.publish(twist)
        cv2.imshow("AprilTags", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraAprilTagNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
