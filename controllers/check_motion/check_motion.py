from controller import Supervisor, AnsiCodes, Node
import math
import numpy as np
import random
import time
import random
import numpy as np

MOTOR_NAMES = [
    "right_ankle_roll_joint",                       # ID1
    "right_ankle_pitch_joint",                      # ID2
    "right_knee_pitch_joint",                       # ID3
    "right_waist_pitch_joint",                      # ID4
    "right_waist_roll_joint [hip]",                 # ID5
    "right_waist_yaw_joint",                        # ID6
    "right_shoulder_pitch_joint [shoulder]",        # ID7
    "right_shoulder_roll_joint",                    # ID8
    "right_elbow_pitch_joint",                      # ID9
    "left_ankle_roll_joint",                        # ID10
    "left_ankle_pitch_joint",                       # ID11
    "left_knee_pitch_joint",                        # ID12
    "left_waist_pitch_joint",                       # ID13
    "left_waist_roll_joint [hip]",                  # ID14
    "left_waist_yaw_joint",                         # ID15
    "left_shoulder_pitch_joint [shoulder]",         # ID16
    "left_shoulder_roll_joint",                     # ID17
    "left_elbow_pitch_joint",                       # ID18
    "head_yaw_joint"                                # ID19
]

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())
player = supervisor.getFromDef('PLAYER')

#motor 
__motors = []
for name in MOTOR_NAMES:
    __motors.append(supervisor.getDevice(name))

# motor random
for motor in __motors:
    initial_position = np.random.uniform(low=-np.pi, high=np.pi)
    motor.setPosition(initial_position)

# children = player.getProtoField('children')
# for i in range(children.getCount()):
#     child = children.getMFNode(i)