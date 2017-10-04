import numpy as np
import time, math

from dronekit import connect
from pymavlink import mavutil

class ControllerBase:
    def step(self):
        raise NotImplementedError

class SimpleVelocityController(ControllerBase):

    currentPose = np.array([0,0,0,0,0,0])
    PID_xy = np.array([0,0,0])
    PID_z = np.array([0,0,0])
    velocity = np.array([0,0,0])
    lasttime = 0
    lastPose = np.array([0,0,0,0,0,0])
    targetPosition = np.array([0,0,0])
    error = np.zeros((3,3))

    def __init__(self, cfg, logger, vehicle):
        self.lasttime = time.time()
        self.cfg = cfg
        self.logger = logger
        self.vehicle = vehicle

    def to_quaternion(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Convert degrees to quaternions
        """
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    def step(self, currentPose, targetPosition):

        self.currentPose = np.array(currentPose)
        self.targetPosition = np.array(targetPosition)

        # Calculate the velocity
        currTime = time.time()
        self.velocity = (self.currentPose[:3] - self.lastPose[:3]) / (currTime - self.lasttime)

        self.lasttime = currTime
        self.lastPose = self.currentPose

        gPos = self.currentPose[:3]
        theta = -self.currentPose[5]    # yaw correction in radians
        rotZ = np.array(
                        [[ np.cos(theta), -np.sin(theta), 0],
                         [ np.sin(theta),  np.cos(theta), 0],
                         [ 0            ,  0            , 1]]
                        )

        # Calculate the 3D error vector with corrected rotation
        distToTarget = rotZ.dot(self.targetPosition - gPos)

        # Set the desired speed based on the predefined pattern
        desiredVelocity = 1 / (1 + np.exp(-5*np.absolute(distToTarget)+5))
        desiredVelocity = desiredVelocity * distToTarget / np.absolute(distToTarget)

        p_e = desiredVelocity - self.velocity
        p_e[2] = distToTarget[2]

        # Update the error matrix
        err = self.error
        err = np.array([
                        [p_e[0], err[0,1]+p_e[0], p_e[0]-err[0,0]],
                        [p_e[1], err[1,1]+p_e[1], p_e[1]-err[1,0]],
                        [p_e[2], err[2,1]+p_e[2], p_e[2]-err[2,0]]
        ])
        self.error = err

        # Generate and set the target angle for XY plane
        angle = err[:2,:].dot(self.cfg['tuning']['PID_xy'])
        thrust_add = err[2,:].dot(self.cfg['tuning']['PID_z'])

        roll_angle  = -angle[1]
        pitch_angle = -angle[0]
        thrust = 0.5 + thrust_add

        #print 'roll: {}\npitch: {}\nthrust: {}'.format(roll_angle, pitch_angle, thrust)

        # Use defaults for the others
        yaw_rate = 0.0

        # Non-linear cutoff for safety
        if self.cfg['tuning']['NONLIN_SAFETY']:
            if roll_angle  > self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = self.cfg['tuning']['MAX_SAFE_ANGLE']
            if pitch_angle > self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = self.cfg['tuning']['MAX_SAFE_ANGLE']
            if roll_angle  < -self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = -self.cfg['tuning']['MAX_SAFE_ANGLE']
            if pitch_angle < -self.cfg['tuning']['MAX_SAFE_ANGLE']:
                roll_angle = -self.cfg['tuning']['MAX_SAFE_ANGLE']

        msg = self.vehicle.message_factory.set_attitude_target_encode(
                     0,
                     0,                                         #target system
                     0,                                         #target component
                     0b00000000,                                #type mask: bit 1 is LSB
                     self.to_quaternion(roll_angle, pitch_angle),    #q
                     0,                                         #body roll rate in radian
                     0,                                         #body pitch rate in radian
                     math.radians(yaw_rate),                    #body yaw rate in radian
                     thrust)                                    #thrust
        self.vehicle.send_mavlink(msg)
