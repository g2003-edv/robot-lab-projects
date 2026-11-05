#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quadruped Robot Gait Controller
Implements Trot gait with foot trajectory interpolation.
Publishes joint angles to /joint_states at 50Hz.
"""

import os
import sys

import rospy
import numpy as np
from sensor_msgs.msg import JointState

# kinematics.py is a library that is installed next to this node (not as an
# executable).  Adding this file's directory keeps `import kinematics` working
# from both the catkin devel-space relay scripts and the install space.
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from kinematics import QuadrupedKinematics  # noqa: E402

# After this many consecutive IK failures the node stops instead of publishing
# a made-up pose forever.
MAX_CONSECUTIVE_IK_FAILURES = 5


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
        # Neutral stance: foot L1 + stance_radius away from the hip, body_height
        # below it.  With L2 = L3 = 0.10 the leg can reach
        # sqrt((stance_radius + step_length/2)^2 + body_height^2) = 0.199 m,
        # just inside L2 + L3 = 0.2 m, so the whole step stays solvable.
        self.stance_radius = 0.055    # m
        self.body_height = 0.180      # m

        # Last valid solution, held while IK is temporarily unsolvable.
        self._last_angles = None
        self._ik_failures = 0
        self._fatal = False

    def foot_trajectory(self, phase):
        """
        Cycloid foot trajectory for swing phase.
        phase: 0->1 within one swing period.
        Returns (x, z) displacement relative to the start of the swing, with
        x going from 0 to step_length and z lifting by step_height.
        """
        x = self.step_length * (phase - np.sin(2 * np.pi * phase) / (2 * np.pi))
        z = self.step_height * (1 - np.cos(2 * np.pi * phase)) / 2
        return x, z

    def foot_position(self, leg_phase):
        """
        Foot displacement (fx, fz) relative to the neutral stance position.

        Stance: the foot is planted, so in the body frame it travels backwards
        from +step_length/2 to -step_length/2 while the body moves forward over
        it.  Swing: the foot lifts and returns from -step_length/2 to
        +step_length/2.  The two phases therefore meet at the same position and
        the trajectory is continuous over a whole cycle.
        """
        if leg_phase < self.duty_factor:
            progress = leg_phase / self.duty_factor
            fx = self.step_length / 2.0 - self.step_length * progress
            fz = 0.0
        else:
            swing_progress = (leg_phase - self.duty_factor) / (1.0 - self.duty_factor)
            dx, dz = self.foot_trajectory(swing_progress)
            fx = -self.step_length / 2.0 + dx
            fz = dz
        return fx, fz

    def compute_joint_angles(self):
        """
        Compute joint angles for all 4 legs at current time.

        Returns a list of 12 angles, the last valid list if IK fails for a
        single cycle, or None when there is nothing safe to publish yet / the
        controller has to stop (self._fatal).
        """
        t = rospy.get_time()
        cycle_phase = (t % self.cycle_time) / self.cycle_time

        angles = []
        # Trot: diagonal legs move together
        # FL & RR in phase, FR & RL in anti-phase
        diagonal_pairs = [(0, 3), (1, 2)]

        for leg_idx in range(4):
            pair = 0 if leg_idx in diagonal_pairs[0] else 1
            leg_phase = (cycle_phase + pair * 0.5) % 1.0

            fx, fz = self.foot_position(leg_phase)
            neutral_x, neutral_y = self.kin.neutral_foot_position(
                leg_idx, self.stance_radius)

            joint_angles = self.kin.inverse_kinematics(
                leg_idx, neutral_x + fx, neutral_y, -self.body_height + fz)

            if joint_angles is None:
                self._ik_failures += 1
                rospy.logwarn(
                    'IK failed for leg %s (%d consecutive failure(s))',
                    self.leg_names[leg_idx], self._ik_failures)
                if self._ik_failures >= MAX_CONSECUTIVE_IK_FAILURES:
                    rospy.logerr(
                        'IK failed %d consecutive times, stopping the gait controller',
                        self._ik_failures)
                    self._fatal = True
                    return None
                if self._last_angles is None:
                    # Nothing valid has been computed yet: publish nothing
                    # rather than driving the legs to an arbitrary pose.
                    return None
                # Hold the previous valid pose instead of a fixed fallback.
                return list(self._last_angles)

            angles.extend(joint_angles)

        self._ik_failures = 0
        self._last_angles = list(angles)
        return angles

    def run(self):
        rospy.loginfo("Trot gait controller started.")
        while not rospy.is_shutdown():
            angles = self.compute_joint_angles()
            if angles is None:
                if self._fatal:
                    break
                self.rate.sleep()
                continue

            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            msg.name = self.joint_names
            msg.position = angles
            self.pub.publish(msg)
            self.rate.sleep()
        rospy.loginfo("Trot gait controller stopped.")


if __name__ == '__main__':
    rospy.init_node('trot_gait_controller')
    controller = TrotGaitController()
    controller.run()
