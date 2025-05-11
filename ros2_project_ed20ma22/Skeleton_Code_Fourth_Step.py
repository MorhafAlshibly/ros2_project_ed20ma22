# Exercise 4 - following a colour (blue) and stopping when too close.

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # Initialise a publisher to publish messages to the robot base
        # We covered which topic receives messages that move the robot in the 3rd Lab Session
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz


        # Initialise any flags that signal a colour has been detected (default to false)
        self.blue_found = False
        self.too_close = False


        # Initialise the value you wish to use for sensitivity in the colour detection (10 should be enough)
        self.sensitivity = 10


        # Initialise some standard movement messages such as a simple move forward and a message with all zeroes (stop)

        # Remember to initialise a CvBridge() and set up a subscriber to the image topic you wish to use
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # We covered which topic to subscribe to should you wish to receive image data

    def callback(self, data):

        # Convert the received image into a opencv image
        # But remember that you should always wrap a call to this conversion method in an exception handler
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        

        # Set the upper and lower bounds for the two colours you wish to identify
        #hue value = 0 to 179
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        
        #hsv_colour1_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour1_upper = np.array([<Hue value> + self.sensitivity, 255, 255])
        
        #hsv_colour2_lower = np.array([<Hue value> - self.sensitivity, 100, 100])
        #hsv_colour2_upper = np.array([<Hue value> + self.sensitivity, 255, 255])

        # Convert the rgb image into a hsv image
        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour using the cv2.inRange() method
        green_image = cv2.inRange(Hsv_image, hsv_green_lower, hsv_green_upper)
        red_image = cv2.inRange(Hsv_image, hsv_red_lower, hsv_red_upper)
        blue_image = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)


        # Apply the mask to the original image using the cv2.bitwise_and() method
        # As mentioned on the worksheet the best way to do this is to bitwise and an image with itself and pass the mask to the mask parameter
        mask_image = cv2.bitwise_or(green_image, red_image)
        mask_image = cv2.bitwise_or(mask_image, blue_image)
        final_image = cv2.bitwise_and(image, image, mask=mask_image)

        # Find the contours that appear within the certain colour mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE

        # Loop over the contours
        if len(contours)>0:

            # There are a few different methods for identifying which contour is the biggest
            # Loop through the list and keep track of which contour is biggest or
            # Use the max() method to find the largest contour
            
            c = max(contours, key=cv2.contourArea)


            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            print("Area of contour: ", cv2.contourArea(c))
            if cv2.contourArea(c) > 100: #<What do you think is a suitable area?>
                # Alter the value of the flag
                (x, y), radius = cv2.minEnclosingCircle(c)
                center = (int(x), int(y))
                radius = int(radius)
                
                cv2.circle(image, center, radius, (0, 255, 0), 2)
                
                self.blue_found = True
            else:
                self.blue_found = False
                
        
        if self.blue_found:
            print("Blue object detected")

        #Check if a flag has been set = colour object detected - follow the colour object
        if self.blue_found:
            if cv2.contourArea(c) > 30000:
                # Too close to object, need to move backwards
                # Set a flag to tell the robot to move backwards when in the main loop
                print("Too close to object")
                self.too_close = True
                
            elif cv2.contourArea(c) < 30000:
                # Too far away from object, need to move forwards
                # Set a flag to tell the robot to move forwards when in the main loop
                print("Too far from object")
                self.too_close = False
        #else:
                

            # Be sure to do this for the other colour as well
            # Setting the flag to detect blue, and stop the turtlebot from moving if blue is detected
        
        cv2.namedWindow('threshold_Feed2',cv2.WINDOW_NORMAL) 
        cv2.imshow('threshold_Feed2', image)
        cv2.resizeWindow('threshold_Feed2',320,240)
        cv2.waitKey(3)



        # Show the resultant images you have created. You can show all of them or just the end result if you wish to.

    def walk_forward(self):
        #Use what you learnt in lab 3 to make the robot move forwards
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Set forward speed


        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def walk_backward(self):
        # Use what you learnt in lab 3 to make the robot move backwards
        desired_velocity = Twist()
        desired_velocity.linear.x = -0.2


        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_velocity)
            self.rate.sleep()

    def stop(self):
        # Use what you learnt in lab 3 to make the robot stop
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0


        self.publisher.publish(desired_velocity)

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    


    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            # Publish moves
            #if found green:
            #    if robot is too close:
            #        move robot backward()
            #    else:
            #        robot walk forward()
            if robot.blue_found == True:
                if robot.too_close == True:
                    robot.walk_backward()
                else:
                    robot.walk_forward()
            #else:
            #    robot.stop()

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
