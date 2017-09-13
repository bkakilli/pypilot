import numpy as np
import time

class SimpleVelocityController:

    currentPose = np.array([0,0,0,0,0,0])
    PID_xy = np.array([0,0,0])
    PID_z = np.array([0,0,0])
    velocity = np.array([0,0,0])
    lasttime = 0
    lastPose = np.array([0,0,0,0,0,0])
    targetPosition = np.array([0,0,0])
    error = np.zeros((3,3))

    def __init__(self, PID_xy, PID_z):
        self.lasttime = time.time()
        self.PID_xy = PID_xy
        self.PID_z = PID_z

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
        angle = err[:2,:].dot(self.PID_xy)
        thrust_add = err[2,:].dot(self.PID_z)

        roll_angle  = angle[0]
        pitch_angle = -angle[1]
        thrust = 0.5 + thrust_add

        # Use defaults for the others
        yaw_rate = 0.0

        return {'roll_angle': roll_angle, 'pitch_angle': pitch_angle, 'thrust': thrust, 'yaw_rate': yaw_rate}
