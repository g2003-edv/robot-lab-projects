#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quadruped Robot Gait Controller
Implements Trot gait with foot trajectory interpolation.
Publishes joint angles to /joint_states at 50Hz.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from kinematics import QuadrupedKinematics

class TrotGaitController:
    def __init__(self):
        self.kin = QuadrupedKinematics()
        self.leg_names = ['FL', 'FR', 'RL', 'RR']
        self.joint_names = []
        for leg in self.leg_names:
            self.joint_names += [f'{leg}_hip', f'{leg}_thigh', f'{leg}_calf']

        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.rate = rospy.Rate(50)

        # Gait parameters
        self.step_height = 0.030      # m
        self.step_length = 0.060      # m
        self.cycle_time = 0.8         # s per gait cycle
        self.duty_factor = 0.5        # stance phase ratio

    def foot_trajectory(self, phase):
        """
        Cycloid foot trajectory for swing phase.
        phase: 0->1 within one swing period.
        Returns (x, z) displacement relative to neutral position.
        """
        x = self.step_length * (phase - np.sin(2 * np.pi * phase) / (2 * np.pi))
        z = self.step_height * (1 - np.cos(2 * np.pi * phase)) / 2
        return x, z

    def compute_joint_angles(self):
        """Compute joint angles for all 4 legs at current time."""
        t = rospy.get_time()
        cycle_phase = (t % self.cycle_time) / self.cycle_time

        angles = []
        # Trot: diagonal legs move together
        # FL & RR in phase, FR & RL in anti-phase
        diagonal_pairs = [(0, 3), (1, 2)]

        for leg_idx in range(4):
            pair = 0 if leg_idx in diagonal_pairs[0] else 1
            leg_phase = (cycle_phase + pair * 0.5) % 1.0

            # Swing vs Stance
            if leg_phase < self.duty_factor:
                # Stance: foot on ground, body moves forward
                progress = leg_phase / self.duty_factor
                fx = -self.step_length / 2 + self.step_length * progress
                fz = -0.18  # neutral height
            else:
                # Swing: foot lifts and moves forward
                swing_progress = (leg_phase - self.duty_factor) / (1 - self.duty_factor)
                dx, dz = self.foot_trajectory(swing_progress)
                fx = self.step_length / 2 - dx
                fz = -0.18 + dz

            # IK to joint angles
            joint_angles = self.kin.inverse_kinematics(leg_idx, fx, 0.0, fz)
            if joint_angles is None:
                joint_angles = (0.0, -0.3, 0.6)  # fallback neutral pose
            angles.extend(joint_angles)

        return angles

    def run(self):
        rospy.loginfo("Trot gait controller started.")
        while not rospy.is_shutdown():
            angles = self.compute_joint_angles()
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = angles
            self.pub.publish(msg)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('trot_gait_controller')
    controller = TrotGaitController()
    controller.run()
