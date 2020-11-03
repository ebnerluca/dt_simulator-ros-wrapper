#!/usr/bin/env python3

import os
import rospy
import yaml
import copy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped
import gym_duckietown
from gym_duckietown.simulator import Simulator
import numpy as np
import cv2
from cv_bridge import CvBridge

class DTSimulatorRosWrapper(DTROS):


    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DTSimulatorRosWrapper, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        rospy.loginfo("[DTSimulatorRosWrapper]: __init__() ...")

        #read parameters
        self.vehicleName_ = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        rospy.loginfo("[DTSimulatorRosWrapper]:    vehicleName_: %s" %self.vehicleName_)

        #construct image publisher
        imageTopic = self.vehicleName_ + "/camera_node/image/compressed" #TODO
        self.imagePublisher_ = rospy.Publisher(imageTopic, CompressedImage, queue_size=10) #TODO: adjust message type, adjust topic
        rospy.loginfo("[DTSimulatorRosWrapper]:    Publishing images to: %s" %imageTopic)

        #construct wheel cmd subscriber
        wheelCommandsTopic = self.vehicleName_ + "/wheels_driver_node/wheels_cmd"
        self.wheelCommandsSubscriber_ = rospy.Subscriber(wheelCommandsTopic, WheelsCmdStamped, self.wheelCommandsSubscriberCallback)
        rospy.loginfo("[DTSimulatorRosWrapper]:    Subscribing wheel commands from: %s" %wheelCommandsTopic)
        self.wheelCommands_ = [0.1,0.1] #initialize wheelCommands

        #initialize simulator
        rospy.loginfo("[DTSimulatorRosWrapper]:    Initializing Simulator ...")
        self.env_ = Simulator(
                seed=123, # random seed
                map_name="loop_empty",
                max_steps=500001, # we don't want the gym to reset itself
                domain_rand=0,
                camera_width=640,
                camera_height=480,
                accept_start_angle_deg=4, # start close to straight
                full_transparency=True,
                distortion=True,
            )
        self.bridge = CvBridge()
        rospy.loginfo("[DTSimulatorRosWrapper]:    Initialized Simulator")

        #camera info
        rospy.loginfo("[DTSimulatorRosWrapper]:    Loading camera calibration ...")
        # For intrinsic calibration
        self.cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        self.frame_id = rospy.get_namespace().strip('/') + '/camera_optical_frame'
        self.cali_file = self.cali_file_folder + "fakebot.yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (self.cali_file_folder + "default.yaml")
            rospy.loginfo ("Calibration file: %s" %self.cali_file)

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.logwarn("No calibration file found in " + self.cali_file)
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        #self.update_camera_params()
        self.log("Using calibration file: %s" % self.cali_file)

        self.pub_camera_info = rospy.Publisher(self.vehicleName_ + "/camera_node/camera_info", CameraInfo, queue_size=1)

        rospy.loginfo("[DTSimulatorRosWrapper]:    Loaded camera calibration.")

        rospy.loginfo("[DTSimulatorRosWrapper]: done.")

    
    def run(self):

        while not rospy.is_shutdown():
            self.updateSimulator()


    def wheelCommandsSubscriberCallback(self, data):

        #get wheel commands
        self.wheelCommands_[0] = data.vel_left
        self.wheelCommands_[1] = data.vel_right


    def publishImage(self, observation):
        
        #create image from observation
        imgArray = np.asarray(observation)
        cv2.imwrite('frame.jpg', imgArray)
        img = cv2.imread('frame.jpg', -1) #-1: read file unchanged
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        stamp = rospy.Time.now()
        compressedImgMsg = self.bridge.cv2_to_compressed_imgmsg(img,dst_format='jpg')
        compressedImgMsg.header.stamp = stamp
        self.imagePublisher_.publish(compressedImgMsg)

        # Publish the CameraInfo message
        self.current_camera_info.header.stamp = stamp
        self.pub_camera_info.publish(self.current_camera_info)


    def updateSimulator(self):

        #uptade simulator
        observation, reward, done, misc = self.env_.step(self.wheelCommands_) #TODO
        self.env_.render()
        if done:
            self.env_.reset()
            rospy.loginfo("[DTSimulatorRosWrapper]: Simulator reset.")

        self.publishImage(observation)


    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.
        Loads the intrinsic and extrinsic camera matrices.
        Args:
            filename (:obj:`str`): filename of calibration files.
        Returns:
            :obj:`CameraInfo`: a CameraInfo message object
        """
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info


if __name__ == '__main__':
    # create the node
    node = DTSimulatorRosWrapper(node_name='dt_simulator_ros_wrapper_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()