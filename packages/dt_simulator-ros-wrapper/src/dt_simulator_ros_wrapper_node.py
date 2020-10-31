#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
#from duckietown_msgs.msg import SegmentList, Segment, BoolStamped
import gym_duckietown
#from gym_duckietown.simulator import Simulator

class DTSimulatorRosWrapper(DTROS):



    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(DTSimulatorRosWrapper, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        rospy.loginfo("[DTSimulatorRosWrapper]: __init__() ...")

        #read parameters
        vehicleName_ = os.environ.get('VEHICLE_NAME', 'default_vehicle')
        rospy.loginfo("[DTSimulatorRosWrapper]:    vehicleName_: %s" %vehicleName_)

        #construct publisher
        imageTopic = vehicleName_ + "/imageTopic" #TODO
        self.imagePublisher_ = rospy.Publisher(imageTopic, String, queue_size=10) #TODO: adjust message type, adjust topic
        rospy.loginfo("[DTSimulatorRosWrapper]:    Publishing images to: %s" %imageTopic)

        #construct Subscriber
        wheelCommandsTopic = vehicleName_ + "wheelCommandsTopic"
        self.wheelCommandsSubscriber_ = rospy.Subscriber(wheelCommandsTopic, String, self.wheelCommandsSubscriberCallback)
        rospy.loginfo("[DTSimulatorRosWrapper]:    Subscribing wheel commands from: %s" %wheelCommandsTopic)
        self.wheelCommands_ = [0.1,0.1] #initialize wheelCommands

        rospy.loginfo("[DTSimulatorRosWrapper]: ... done.")

        #env = Simulator(
        #        seed=123, # random seed
        #        map_name="loop_empty",
        #        max_steps=500001, # we don't want the gym to reset itself
        #        domain_rand=0,
        #        camera_width=640,
        #        camera_height=480,
        #        accept_start_angle_deg=4, # start close to straight
        #        full_transparency=True,
        #        distortion=True,
        #    )   
    
    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():

            self.updateSimulator()
            rate.sleep()

    def wheelCommandsSubscriberCallback(self, data):
        rospy.loginfo("[DTSimulatorRosWrapper]: Wheel Commands: %s", data.data)

        #get wheel commands
        #self.wheelCommands_ = data #TODO


    def updateSimulator(self):
        rospy.loginfo("[DTSimulatorRosWrapper]: Updating simulator ...")

        #get image from simulator
        #observation, reward, done, misc = env.step(wheelCommands_) #TODO
        #env.render()
        #if done:
        #    env.reset()

        #publish image on image topic
        imagePlaceholder = "This should be an image."
        self.imagePublisher_.publish(imagePlaceholder) #TODO, adjust message
        rospy.loginfo("[DTSimulatorRosWrapper]:    Published image: %s" %imagePlaceholder)

        rospy.loginfo("[DTSimulatorRosWrapper]: ... done")

if __name__ == '__main__':
    # create the node
    node = DTSimulatorRosWrapper(node_name='dt_simulator_ros_wrapper_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()