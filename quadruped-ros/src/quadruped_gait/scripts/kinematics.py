#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quadruped Robot Kinematics
- Forward Kinematics: joint angles -> foot tip position
- Inverse Kinematics: foot tip position -> joint angles
"""

import numpy as np

class QuadrupedKinematics:
    def __init__(self):
        # Leg link lengths (unit: m)
        self.L1 = 0.045   # Coxa  length (hip offset)
        self.L2 = 0.100   # Femur length (upper leg)
        self.L3 = 0.100   # Tibia length (lower leg)

    def forward_kinematics(self, leg_id, theta1, theta2, theta3):
        """
        Forward kinematics for one leg.
        theta1: hip yaw, theta2: hip pitch, theta3: knee pitch
        Returns foot tip position (x, y, z) in body frame.
        """
        x = (self.L2 * np.cos(theta2) + self.L3 * np.cos(theta2 + theta3)) * np.cos(theta1)
        y = (self.L2 * np.cos(theta2) + self.L3 * np.cos(theta2 + theta3)) * np.sin(theta1)
        z = self.L2 * np.sin(theta2) + self.L3 * np.sin(theta2 + theta3)
        return np.array([x, y, z])

    def inverse_kinematics(self, leg_id, x, y, z):
        """
        Inverse kinematics for one leg.
        Given foot tip position (x, y, z), compute joint angles.
        Returns (theta1, theta2, theta3) or None if unreachable.
        """
        try:
            theta1 = np.arctan2(y, x)
            r = np.sqrt(x**2 + y**2) - self.L1
            d = np.sqrt(r**2 + z**2)

            if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
                return None  # Unreachable

            cos_theta3 = (r**2 + z**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            theta3 = -np.arccos(np.clip(cos_theta3, -1.0, 1.0))

            alpha = np.arctan2(z, r)
            beta = np.arccos(np.clip((self.L2**2 + d**2 - self.L3**2) / (2 * self.L2 * d), -1.0, 1.0))
            theta2 = alpha + beta

            return (theta1, theta2, theta3)
        except Exception:
            return None
