from geometry_msgs.msg import Pose, PoseArray, Quaternion
from pf_base import PFLocaliserBase
from random import gauss
from random import uniform
from random import choice
from random import seed
import numpy as np
import copy
import math
import rospy
from itertools import repeat

from util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):

    PARTICLE_COUNT = 1024

    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()

        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 1  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1  # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 1  # Odometry y axis (side-side) noise
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
        particlecloud = PoseArray()
        seed(100)
        avg = 0
        for i in range(0, self.PARTICLE_COUNT):
            newpose = self.new_pose_with_error(initialpose.pose.pose, None, 0.7)
            particlecloud.poses.append(copy.deepcopy(newpose))
            avg += newpose.position.x
        #print(particlecloud)
        #print(avg / self.PARTICLE_COUNT)
        return particlecloud

    def new_pose_with_error(self, pose, scan, position_noise, turn_noise=1.0):
        on_map = True
        newpose = Pose()
        newpose = copy.deepcopy(pose)
        newpose.position.x += gauss(0, 1) * position_noise
        newpose.position.y += gauss(0, 1) * position_noise
        newpose.orientation = rotateQuaternion(newpose.orientation, gauss(0, 1) * turn_noise)
        return newpose

    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.

        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """


        weights = []
        scan.ranges = map(lambda x: scan.range_max if math.isnan(x) else x, scan.ranges)
        for particle in self.particlecloud.poses:
            weights.append(self.sensor_model.get_weight(scan,particle))
        """
        weights = weights / np.sum(weights)

        weightsi = []
        weightsj = []
        score = np.zeros(len(self.particlecloud.poses))
        cum_i = 0
        cum_j = 0

        for i in range(len(weights)):
            weightsi.append(cum_i)
            cum_i += weights[i]
            cum_j += weights[i]
            weightsj.append(cum_j)

        for particle in range(0,self.PARTICLE_COUNT):
            r = uniform(0,1)
            for m in range(len(weights)):
                if((r >= float(weightsi[m])) & (r < float(weightsj[m]))):
                    score[m] += 1
        particlecloud = PoseArray()
        particlecloud = [x for index,item in enumerate(self.particlecloud.poses) for x in repeat(item, int(score[index]))]
        """

        #mattys test
        weights = weights / np.min(weights) #smallest weight is now 1
        print(weights)
        bigset = []
        for i in range(self.PARTICLE_COUNT):
            c = 0
            while(c < weights[i]):
                bigset.append(self.particlecloud.poses[i])
                c += 1
        particlecloud = PoseArray()
        print(len(bigset))
        for i in range(self.PARTICLE_COUNT):
            particlecloud.poses.append(self.new_pose_with_error(choice(bigset), scan, 0.3))
        self.particlecloud = particlecloud
        print("done")
        print(len(particlecloud.poses))
        return particlecloud

    def estimate_pose_impl_average(self):
        # Just average everything
        cumulative_pose = Pose()

        for particle in self.particlecloud.poses:
            cumulative_pose.position.x += particle.position.x
            cumulative_pose.position.y += particle.position.y
            cumulative_pose.position.z += particle.position.z

            cumulative_pose.orientation.x += particle.orientation.x
            cumulative_pose.orientation.y += particle.orientation.y
            cumulative_pose.orientation.z += particle.orientation.z
            cumulative_pose.orientation.w += particle.orientation.w

        num_particles = len(self.particlecloud.poses)
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
        return self.estimate_pose_impl_average() # quick solution - change
