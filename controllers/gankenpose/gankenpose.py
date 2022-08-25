# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#from gamestate import GameState
from field import Field
#from forceful_contact_matrix import ForcefulContactMatrix

from controller import Supervisor, AnsiCodes, Node

import copy
import json
import math
import numpy as np
import os
import random
import socket
import subprocess
import sys
import time
import traceback
import transforms3d
import random
import numpy as np
import time
import csv  # csvファイルの操作を楽にしてくれる
from scipy.spatial import ConvexHull
from types import SimpleNamespace
from skopt import gp_minimize

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())
field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.20 0.1 0.450 rotation 0 0 1 0}}')
player = supervisor.getFromDef('PLAYER')
ball = supervisor.getFromDef('BALL')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
player_controller = supervisor.getFromDef('PLAYER').getField('controller')
ball_translation = supervisor.getFromDef('BALL').getField('translation')
ball_rotation = supervisor.getFromDef('BALL').getField('rotation')


count = 0
while supervisor.step(time_step) != -1:
    count += 1
    if count > 10000:
        break
    if count == 1000:
        player_rotation = player_rotation.getSFRotation()
        player_translation = player_translation.getSFVec3f()
        print(player_rotation)
        print(player_translation)