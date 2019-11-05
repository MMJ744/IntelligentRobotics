from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
from random import gauss
from random import seed
import math
import rospy

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):

    PARTICLE_COUNT = 250

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        global PARTICLE_COUNT
        noise = 1
        seed(100)

        for i in range(0, PARTICLE_COUNT):
            newpose = initialpose
            newpose.pose.pose.position.x += gauss(0,3)*noise
            newpose.pose.pose.position.y += gauss(0,3)*noise
            newpose.pose.pose.orientation = rotateQuaternion(newpose.pose.pose.orientation,gauss(0,1))
            self.particlecloud[i] = newpose
        pass


    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        global PARTICLE_COUNT
        particlecloud = set()
        previous = 0
        cumulative = []

        for particle in self.particlecloud:
            cumulative.append(previous + self.sensor_model.get_weight(self, scan, particle))
            previous = cumulative[-1]

        cumulative = map(lambda x: x / previous, cumulative)

        uniform = range(PARTICLE_COUNT) / PARTICLE_COUNT

        i=0
        for j in range(1,PARTICLE_COUNT):
            while uniform[j] > cumulative[i]:
                i = i+1
            # particlecloud.add() //particle generate here
            uniform[j+1] = uniform[j] + 1/PARTICLE_COUNT

        self.particlecloud = particlecloud
        pass

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        pass
