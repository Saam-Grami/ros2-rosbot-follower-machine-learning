
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image #This is for importing the images from the camera
from geometry_msgs.msg import Twist #This is for publishing the geo msgs for cmd_vel/
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import os


class ModelNode (Node): # MODIFY NAME
    def __init__(self):
        super().__init__("model_node")
        self.get_logger().info("Model node!")#This line lets you print to the terminal

        model_path = '/home/regina2004/mtre4820_final_project/src/ml_model/ml_model/ambulance_model_savedmodel'
        self.model = tf.keras.layers.TFSMLayer(model_path, call_endpoint='serving_default')
        self.get_logger().info("CNN model loaded")

        self.bridge = CvBridge()

        self.subscriber_ = self.create_subscription(Image, "/rosbot/camera/image_raw", self.listener_callback, 10)#needs to take in camera matrix
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

    def listener_callback(self, msg):#This run the other functions we add, like cnn_model
        #Subscriber
        self.get_logger().info("Message received!")  

        #Publisher
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = self.cnn_model(msg)
        self.publisher_.publish(cmd) 

    def preprocess_image(self, cv_image):
           img = cv2.resize(cv_image, (224, 224))
           img = img / 255
           img = np.expand_dims(img, axis=0)
           return img.astype(np.float32)

    def cnn_model(self, msg):
        self.get_logger().info("CNN!")
        #Converts ROS imaghe message into a bgr image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #Preproecces to match training
        processed = self.preprocess_image(cv_image)
        #Grabs the scalar
        prediction = self.model(processed, training=False)
        output = list(prediction.values())[0]
        ang_vel = float(output[0][0])
        #Clamp the motion for the angular velocities?
        ang_vel = float(np.clip(ang_vel, -1.0, 1.0))
        #ang_vel = 0.5
        self.get_logger().info(f"Predicted angular velocity: {ang_vel:.4f}")
        return ang_vel

def main(args=None):
    rclpy.init(args=args) 
    node = ModelNode () 
    rclpy.spin(node) 
    rclpy.shutdown()  

if __name__ == "__main__": 
	main() 
