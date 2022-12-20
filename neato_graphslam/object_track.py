import numpy as np
import cv2 as cv
from object_class import objectdetect
import os
import rclpy #importing ros
from rclpy.node import Node
from getch import getch
import sys
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from threading import Thread
from threading import Event
from cv_bridge import CvBridge
import time


class objecttrack(Node):
    """
    ROS Node that attempts to provide an all in one solution
    to real time graph slam. It accomplishes this by running
    multiple threads that include a keyboard input, live video
    processing, and object storage. Intended to work with the
    Neato line of robots.

    Args:

    Returns:
    """

    def __init__(self):

        super().__init__("objecttrack")

        #----------------Keyboard setup section--------------------------

        # print the load screen
        self.load_screen()

        # initializer a publisher for basic keystroke movement
        self.movement = self.create_publisher(Twist, 'cmd_vel', 10)


        # set the angular velocity
        self.angular_velocity = .3

        # set the linear velocity
        self.linear_velocity = .25

        #----------------Image setup section---------------------------
        self.cv_image = None     # the latest image from the camera
        self.draw_image = None   # image copy we will draw crop on
        self.event = Event()     # create an event to stop threads
        self.bridge = CvBridge() # used to convert ROS messages to OpenCV
        self.referpoints = []    # store the reference points for a mouse click
        self.crop = False        # keep track of whether an image is being cropped or not
        # create the topic to listen to video from the Neato
        self.create_subscription(Image, "camera/image_raw", self.process_image, 10)
        self.cv_image = None
        self.thread = Thread(target=self.loop_wrapper)
        self.thread.start()
        self.thread2 = Thread(target=self.transmitkeys, args=(self.event,))
        self.thread2.start()

    def process_image(self, msg):
        """
        Reads the incoming image from the Neato camera
        and converts it to HSV color space.

        Args:
            msg: a ROS object storing the data from the Neato

        Returns:
        """

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # convert the image to HSV color space
        self.cv_image = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

    def loop_wrapper(self):
        """
        Handles running the run_loop() function repeatedly on a separate
        thread in order to avoid any ROS2 bottleneck limitations.

        Args:

        Returns:
        """

        cv.namedWindow('video_window')
        cv.setMouseCallback('video_window', self.click_and_crop)
        while True:
            self.run_loop()
            time.sleep(0.1)

    def transmitkeys(self, event):
        """
        Handles transmitting any robot movement commands based
        on the keyboard input of the user.

        Args:
            event: Thread object that is used to shut down a thread

        Returns:
        """

        while True:
            # check for stop
            if event.is_set():
                break
            # listen for keypress
            key_press = self.listen()



            # make sure there is a keypress
            if key_press is not None:

                if key_press == "W":
                    self.neato_forward()
                elif key_press == "Q":
                    self.turn_left()
                elif key_press == "E":
                    self.turn_right()
                elif key_press == "A":
                    self.neato_stop()
                elif key_press == "D":
                    self.neato_stop()
                elif key_press == "S":
                    self.neato_backward()
                elif key_press == "M":
                    rclpy.shutdown()
                    quit()

    def click_and_crop(self, event, x,y,flags,param):
        """
        Interprets the click and drag mouse interactions of a user to take
        a snippet of an image.

        Args:
            event: type of mouse interaction determined by opencv
            x: x coordinate of where mouse event occurred
            y: y coordinate of where mouse even occurred
            flags: events related to a mouse action
            param: any additional data opencv takes note of

        Returns:
        """

        if event == cv.EVENT_LBUTTONDOWN:
            self.referpoints = [(x,y)] # store the points when left click is detected
            self.crop = True           # indicate image cropping as started

        # check if the left button has been released
        elif event == cv.EVENT_LBUTTONUP:
            # record the ending coordinates of the mouse drag
            self.referpoints.append((x,y))
            self.crop = False # indicate that the crop action has stopped

            # draw a rectangle around the region of interest
            cv.rectangle(self.draw_image, self.referpoints[0], self.referpoints[1], (0, 255, 0), 2)
            cv.imshow('video_window', self.draw_image)




    def run_loop(self):
        """
        Runs all of the key publishing and basic movement commands.

        Args:

        Returns:
        """

        if not self.cv_image is None:
            self.draw_image = cv.cvtColor(self.cv_image, cv.COLOR_HSV2BGR)
            cv.imshow('video_window', self.draw_image)
            cv.waitKey(5)

            if len(self.referpoints) == 2:
                y_points = (self.referpoints[0][1], self.referpoints[1][1])
                x_points = (self.referpoints[0][0], self.referpoints[1][0])
                roi = self.draw_image[min(y_points):max(y_points),min(x_points):max(x_points)]
                cv.imshow("object crop", roi)
                # stop keyboard input for neato control
                self.event.set()
                # clear the terminal
                os.system('cls||clear')
                # ask the user to confirm the image
                confirmation = input("Do you want this image (y/n)? ")
                if confirmation == "y":
                    # record the objects name
                    obj_name = input("What is the object's name? ")
                    # record the objects size in meters
                    obj_size = input("What is the object's width in meters? ")
                    # save the image to the Objects folder
                    cv.imwrite(os.path.join(os.getcwd(), obj_name+"_"+obj_size+".png"), roi)
                    # clear the terminal again, restart neato controls
                    cv.destroyWindow("object crop")
                    os.system('cls||clear')
                    self.load_screen()
                    self.thread.start()
                else:
                    cv.destroyWindow("object crop")
                    os.system('cls||clear')
                    self.load_screen()
                    self.thread.start()




    def turn_left(self):
        """
        Tells the neato to turn counter clockwise. Velocity
        is determined in init.

        Args:

        Returns:

        """
        # create a blank movement ros message
        msg = Twist()
        # populate it with the defined angular velocity
        msg.angular.z = self.angular_velocity

        self.movement.publish(msg)

    def turn_right(self):
        """
        Tells the neato to turn clockwise. Velocity
        is determined in init.

        Args:

        Returns:

        """
        # create a blank movement ros message
        msg = Twist()
        # populate it with the defined angular velocity
        msg.angular.z = -self.angular_velocity

        self.movement.publish(msg)

    def neato_stop(self):
        """
        Tells the neato to stop moving.

        Args:

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()
        # send the message through the publisher
        self.movement.publish(msg)

    def neato_forward(self):
        """
        Tells the neato to move forward

        Args:

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()

        # set the linear velocity
        msg.linear.x = self.linear_velocity

        # send the message through the publisher
        self.movement.publish(msg)

    def neato_backward(self):
        """
        Tells the neato to move backward

        Args:

        Returns:

        """

        # by default, all values for movement message are 0
        msg = Twist()

        # set the linear velocity
        msg.linear.x = -self.linear_velocity

        # send the message through the publisher
        self.movement.publish(msg)

    def load_screen(self):
        """
        Prints a guide of keyboard commands for the Neato.

        Args:

        Returns:
        """

        # create the initial loading screen
        print("_____Controls_____\n" +
               "        ^         |O Add Object in Frame\n" +
               "  <\ Q  W  E />   |P Start or End Slam  \n" +
               "    X  A D  X     |                     \n" +
               "        S         |                     \n" +
               "        v         |M Escape             ")

    def listen(self):
        """
        Listens for a key input from the user, converts the input to uppercase,
        and prints the last keypress in the terminal.

        Args:

        Returns:
            key_hit: an uppercase string representing the key pressed
            by the user in the terminal.
        """

        # listen for the a key hit
        key_hit = getch()

        # convert any input to uppercase
        key_hit = key_hit.upper()

        # print the keystroke while deleting the last one
        sys.stdout.write('\r' + "       {" + key_hit + "}       ")

        return key_hit



def main(args=None):
    rclpy.init(args=args)
    node = objecttrack()
    rclpy.spin(node)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
