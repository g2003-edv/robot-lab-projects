#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Quadruped Robot Kinematics
- Forward Kinematics: joint angles -> foot tip position
- Inverse Kinematics: foot tip position -> joint angles

Frames
------
Body frame: x forward, y left, z up, origin in the body centre.

Leg frame: the coxa (L1) sticks out sideways from the hip mount and defines the
leg's neutral yaw `yaw0` (+pi/2 for the left legs, -pi/2 for the right legs).
`theta1` is the hip yaw measured from that neutral direction, `theta2` is the
femur pitch and `theta3` the tibia pitch (0 = straight leg, negative = knee
folded).  `leg_id` therefore selects the left/right mirror as well as the hip
mount position -- 0=FL, 1=FR, 2=RL, 3=RR.

All lengths are metres, all angles radians.
"""

import sys

import numpy as np

# Servo travel per joint.
# TODO(verify-on-hardware): these are the mechanical ranges of the leg servos,
# not measured values. Check them on the real robot before the first powered
# run, otherwise a valid IK solution may still drive a servo into its end stop.
JOINT_LIMITS = {
    'hip':   (-np.pi / 2.0, np.pi / 2.0),   # -90 .. +90 deg
    'thigh': (-np.pi / 2.0, np.pi / 2.0),   # -90 .. +90 deg
    'calf':  (-2.6180, 0.0),                # -150 .. 0 deg
}

LEG_NAMES = {0: 'FL', 1: 'FR', 2: 'RL', 3: 'RR'}


def _warn(message):
    """Diagnostics for a library that must stay importable without ROS."""
    print('[kinematics] ' + message, file=sys.stderr)


class QuadrupedKinematics:
    def __init__(self, L1=0.045, L2=0.100, L3=0.100,
                 hip_x=0.080, hip_y=0.050, joint_limits=None):
        # Link lengths (m): coxa (hip offset), femur, tibia.
        self.L1 = float(L1)
        self.L2 = float(L2)
        self.L3 = float(L3)

        # TODO(verify-on-hardware): hip_x / hip_y are taken from the CAD model,
        # not measured. They place the hip yaw joints in the body frame.
        self.legs = {
            0: {'name': 'FL', 'hip_x': +float(hip_x), 'hip_y': +float(hip_y),
                'yaw0': +np.pi / 2.0},
            1: {'name': 'FR', 'hip_x': +float(hip_x), 'hip_y': -float(hip_y),
                'yaw0': -np.pi / 2.0},
            2: {'name': 'RL', 'hip_x': -float(hip_x), 'hip_y': +float(hip_y),
                'yaw0': +np.pi / 2.0},
            3: {'name': 'RR', 'hip_x': -float(hip_x), 'hip_y': -float(hip_y),
                'yaw0': -np.pi / 2.0},
        }

        self.joint_limits = dict(JOINT_LIMITS)
        if joint_limits:
            self.joint_limits.update(joint_limits)

    # ------------------------------------------------------------------ utils
    def _leg(self, leg_id):
        if leg_id not in self.legs:
            raise ValueError(
                'leg_id must be one of {}, got {!r}'.format(sorted(self.legs), leg_id))
        return self.legs[leg_id]

    def _to_leg_frame(self, leg_id, x, y):
        """Body-frame (x, y) -> (theta1 relative to neutral, radial reach).

        The hip mount offset is subtracted first, then the result is rotated by
        the leg's neutral yaw.  This keeps theta1 near 0 for a foot standing
        below its own hip, instead of jumping between 0 and pi as it did when
        atan2() was fed the raw body-frame coordinates.
        """
        leg = self._leg(leg_id)
        dx = float(x) - leg['hip_x']
        dy = float(y) - leg['hip_y']

        theta1 = np.arctan2(dy, dx) - leg['yaw0']
        # Wrap to (-pi, pi] so the hip limit check sees the short way round.
        theta1 = (theta1 + np.pi) % (2.0 * np.pi) - np.pi
        r = np.sqrt(dx * dx + dy * dy) - self.L1
        return float(theta1), float(r)

    def _check_limits(self, leg_id, theta1, theta2, theta3):
        angles = (float(theta1), float(theta2), float(theta3))
        for name, angle in zip(('hip', 'thigh', 'calf'), angles):
            lo, hi = self.joint_limits[name]
            if not (lo <= angle <= hi):
                _warn('leg {} ({}): {} angle {:.4f} rad outside [{:.4f}, {:.4f}]'
                      .format(leg_id, self._leg(leg_id)['name'], name, angle, lo, hi))
                return None
        return angles

    # --------------------------------------------------------------- geometry
    def neutral_foot_position(self, leg_id, stance_radius=0.055):
        """Body-frame foot position for theta1 = 0.

        `stance_radius` is the horizontal femur+tibia reach measured outside the
        coxa, so the neutral foot sits `L1 + stance_radius` away from the hip
        mount along the leg's outward direction.
        """
        leg = self._leg(leg_id)
        radial = self.L1 + float(stance_radius)
        return (leg['hip_x'] + radial * np.cos(leg['yaw0']),
                leg['hip_y'] + radial * np.sin(leg['yaw0']))

    def forward_kinematics(self, leg_id, theta1, theta2, theta3):
        """Joint angles -> foot tip position (x, y, z) in the body frame."""
        leg = self._leg(leg_id)
        radial = (self.L1
                  + self.L2 * np.cos(theta2)
                  + self.L3 * np.cos(theta2 + theta3))
        yaw = leg['yaw0'] + theta1

        x = leg['hip_x'] + radial * np.cos(yaw)
        y = leg['hip_y'] + radial * np.sin(yaw)
        z = self.L2 * np.sin(theta2) + self.L3 * np.sin(theta2 + theta3)
        return np.array([x, y, z])

    def inverse_kinematics(self, leg_id, x, y, z):
        """Foot tip position in the body frame -> (theta1, theta2, theta3).

        Returns None (with a diagnostic on stderr) when the target is outside
        the workspace or the solution violates JOINT_LIMITS.  Callers must
        treat None as a failure, not as "use some default pose".
        """
        theta1, r = self._to_leg_frame(leg_id, x, y)

        if r < 0.0:
            # Foot inside the coxa circle: the planar 2-link solve below would
            # silently mirror the leg through the hip axis.
            _warn('leg {} ({}): r = {:.4f} m < 0, target inside the coxa circle'
                  .format(leg_id, self._leg(leg_id)['name'], r))
            return None

        d = np.sqrt(r * r + float(z) ** 2)
        if d < 1e-6:
            _warn('leg {} ({}): target coincides with the hip joint'
                  .format(leg_id, self._leg(leg_id)['name']))
            return None
        if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
            _warn('leg {} ({}): target out of reach, d = {:.4f} m not in [{:.4f}, {:.4f}]'
                  .format(leg_id, self._leg(leg_id)['name'], d,
                          abs(self.L2 - self.L3), self.L2 + self.L3))
            return None

        cos_theta3 = ((r * r + float(z) ** 2 - self.L2 ** 2 - self.L3 ** 2)
                      / (2.0 * self.L2 * self.L3))
        theta3 = -np.arccos(np.clip(cos_theta3, -1.0, 1.0))

        alpha = np.arctan2(z, r)
        cos_beta = ((self.L2 ** 2 + d ** 2 - self.L3 ** 2)
                    / (2.0 * self.L2 * d))
        beta = np.arccos(np.clip(cos_beta, -1.0, 1.0))
        theta2 = alpha + beta

        return self._check_limits(leg_id, theta1, theta2, theta3)
