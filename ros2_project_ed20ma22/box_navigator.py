import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time

class BoxNavigator(Node):
    
    def __init__(self):
        super().__init__('box_navigator')
        self.navigator = BasicNavigator()
        self.detected_boxes = set()
        self.bridge = CvBridge()
        
        self.red_found = False
        self.green_found = False
        self.blue_found = False

        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.sensitivity = 10
        
        self.get_logger().info("Box navigator initialized.")

    def image_callback(self, data):
        
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        green_image = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        red_image = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)
        
        mask_image = cv2.bitwise_or(green_image, red_image)
        mask_image = cv2.bitwise_or(mask_image, blue_image)
        
        final_image = cv2.bitwise_and(image, image, mask=mask_image)


        self.detected_boxes.clear()  # Clear previously detected boxes
        

        for color_image, color_name in zip([green_image, red_image, blue_image], ['Green', 'Red', 'Blue']):
            contours, _ = cv2.findContours(color_image,mode = cv2.RETR_TREE, method = cv2.CHAIN_APPROX_SIMPLE )
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                if cv2.contourArea(c) > 100:
                    
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    
                    
                    self.detected_boxes.add(color_name)
                
            
        cv2.imshow('Camera', image)
        cv2.waitKey(1)

    def create_pose(self, x, y, yaw):
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def move_to_box(self, color):
        
        self.get_logger().info(f"Moving to the {color} box.")

        box_positions = {
            'Red': self.create_pose(-3.5, -3.4, 0.0),
            'Green': self.create_pose(0.9, -5.4, 0.0),
            'Blue': self.create_pose(-2.5, -8.7, 0.0),
        }

        if color not in box_positions:
            self.get_logger().warn(f"No position defined for {color} box.")
            return

        goal_pose = box_positions[color]
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info(f"Reached {color} box.")

    def navigate(self):
        
        self.navigator.waitUntilNav2Active()

        self.move_to_box('Red')

        if 'Red' in self.detected_boxes and not self.red_found:
            self.get_logger().info("Red box detected!")
            self.red_found = True

        self.move_to_box('Green')

        if 'Green' in self.detected_boxes and not self.green_found:
            self.get_logger().info("Green box detected!")
            self.green_found = True

        self.move_to_box('Blue')

        if 'Blue' in self.detected_boxes and not self.blue_found:
            self.get_logger().info("Blue box detected!")
            self.blue_found = True

def main(args=None):
    rclpy.init(args=args)
    node = BoxNavigator()
    try:
        node.navigate()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
