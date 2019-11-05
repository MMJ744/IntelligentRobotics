from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
from random import gauss
from random import seed
import copy
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
        print(initialpose.pose.pose.position.x)
        avg = 0
        for i in range(0, self.PARTICLE_COUNT):
            newpose = self.new_pose_with_error(initialpose, 1.5)
            self.particlecloud.poses.append(copy.deepcopy(newpose))
            avg += newpose.position.x
        print(avg / self.PARTICLE_COUNT)
        return self.particlecloud

    def new_pose_with_error(self, pose, noise):
        seed(100)
        blank = copy.deepcopy(pose.pose.pose)
        newpose = copy.deepcopy(blank)
        newpose.position.x += gauss(0, 1) * noise
        newpose.position.y += gauss(0, 1) * noise
        newpose.orientation = rotateQuaternion(newpose.orientation, gauss(0, 1))
        return newpose

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """

        particlecloud = PoseArray()
        previous = 0
        cumulative = []

        for particle in self.particlecloud.poses:
            cumulative.append(previous + self.sensor_model.get_weight(self, scan, particle))
            previous = cumulative[-1]

        cumulative = map(lambda x: x / previous, cumulative)

        uniform = map(lambda x: x / self.PARTICLE_COUNT, (self.PARTICLE_COUNT))

        i=0
        for j in range(1, self.PARTICLE_COUNT):
            while uniform[j] > cumulative[i]:
                i = i+1
            newpose = self.new_pose_with_error(particlecloud[i], 1)
            particlecloud.poses.append(newpose)
            uniform[j+1] = uniform[j] + 1/self.PARTICLE_COUNT

        self.particlecloud = particlecloud
        return particlecloud

    def estimate_pose_impl_average(self):
        # Just average everything
        cumulative_pose = Pose()

        for particle in self.particleCloud.poses:
            cumulative_pose.position.x += particle.position.x
            cumulative_pose.position.y += particle.position.y
            cumulative_pose.position.z += particle.position.z

            cumulative_pose.orientation.x += particle.orientation.x
            cumulative_pose.orientation.y += particle.orientation.y
            cumulative_pose.orientation.z += particle.orientation.z
            cumulative_pose.orientation.w += particle.orientation.w

        num_particles = len(self.particleCloud.poses)
        cumulative_pose.position.x /= num_particles
        cumulative_pose.position.y /= num_particles
        cumulative_pose.position.z /= num_particles

        cumulative_pose.orientation.x /= num_particles
        cumulative_pose.orientation.y /= num_particles
        cumulative_pose.orientation.z /= num_particles
        cumulative_pose.orientation.w /= num_particles

        return cumulative_pose


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
        return self.particlecloud[0] # quick solution - change
