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
import const
import csv  # csvファイルの操作を楽にしてくれる
from scipy.spatial import ConvexHull
from types import SimpleNamespace
from skopt import gp_minimize

const.frame_parameter = 12
const.motor_parameter = 18 #首のヨー軸は関係なさそうなので、抜きました

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(
    -1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(
    -1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')
player = supervisor.getFromDef('PLAYER')
ball = supervisor.getFromDef('BALL')
player_translation = supervisor.getFromDef('PLAYER').getField('translation')
player_rotation = supervisor.getFromDef('PLAYER').getField('rotation')
player_controller = supervisor.getFromDef('PLAYER').getField('controller')
ball_translation = supervisor.getFromDef('BALL').getField('translation')
ball_rotation = supervisor.getFromDef('BALL').getField('rotation')

def get_motiondata():
    with open("../play_motion/before_optimize_kick.csv", "r") as f:  # 最適化処理前のモーションファイルを読み込む
        # delimiter = 区切り文字の指定, lineterminator = 改行文字の指定
        read_data = csv.reader(f, delimiter=",", lineterminator="\r\n")
        # リスト内包表記を使用し、read_dataからデータを二次元配列として取得
        data = [row for row in read_data]
        return data

def select_parameter(param, motiondata):
    num = 0
    for frame in range (const.frame_parameter):
        for servo_num in range (const.motor_parameter):
            motiondata[frame][servo_num] = str(int(param[num]))
            num += 1
    return param

def file_update(motiondata):
    with open("../play_motion/motion.csv", "w") as f:  # motion.csvに上書きする(前の状態から更新される)
        writer = csv.writer(f)  # motion.csvに書き込む準備
        writer.writerows(motiondata)  # 1行ずつ書き込み
        print("file write")


def func(param):  # 最適化関数の設定
    global player, ball, children, ball_translation, ball_rotation, supervisor  # グローバル変数の定義
    motiondata = get_motiondata()
    param = select_parameter(param, motiondata)
    file_update(motiondata)

    count = 0
    player.remove()
    children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.2 0.1 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')
    player = supervisor.getFromDef('PLAYER')
    ball.resetPhysics()
    ball_translation.setSFVec3f([0, 0, 0.1])
    ball_rotation.setSFRotation([0, 0, 1, 0])
    while supervisor.step(time_step) != -1:
        count += 1
        if count > 800:
            break
        if count > 800 - 1:
            pos = ball_translation.getSFVec3f()
            with open('result.csv', 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([param[0], param[1], param[2],param[3],param[4], param[5], param[6], param[7], param[8], param[9], param[10], param[11], pos[0], pos[1]])
    return -pos[0]

    # 各パラメータの探索範囲

#各フレームあたりの時間の探索範囲

"""
search_data = get_motiondata()

t_tuple = ()
x_list = []

for i in range(12):
    data = int(search_data[i][0])
    t_tuple[0][0] = (data - 10)
    t_tuple[0][1] = (data + 10)
#    x_list += list(t_tuple)
#    x = tuple(x_list)
print("t_tuple =")
print(t_tuple)
"""

x1 = (90.0, 110.0)
x2 = (-15.0, 5.0)
x3 = (-10.0, 10.0)
x4 = (-26.0, -6.0)
x5 = (11.0, 31.0)
x6 = (-8.5, 11.5)
x7 = (-10.0, 10.0)
x8 = (-28.0, -8.0)
x9 = (-18.0, 2.0)
x10 = (30.0, 50.0)
x11 = (-5.0, 15.0)
x12 = (-10.0, 10.0)
x13 = (-26.0, -6.0)
x14 = (11.0, 31.0)
x15 = (-11.5, 8.5)
x16 = (-10.0, 10.0)
x17 = (-28.0, -8.0)
x18 = (-2.0, 18.0)
x19 = (5.0, 25.0)
x20 = (-24.0, -4.0)
x21 = (-10.0, 10.0)
x22 = (-32.0, -12.0)
x23 = (12.0, 32.0)
x24 = (4.0, 24.0)
x25 = (-8.0, 12.0)
x26 = (-28.0, -8.0)
x27 = (-28.0, -8.0)
x28 = (-10.0, 10.0)
x29 = (-20.0, 0.0)
x30 = (-11.0, 9.0)
x31 = (-30.0, -10.0)
x32 = (10.0, 30.0)
x33 = (4.0, 24.0)
x34 = (-10.0, 10.0)
x35 = (-10.0, 10.0)
x36 = (8.0, 28.0)
x37 = (0.0, 20.0)
x38 = (-24.0, -4.0)
x39 = (-10.0, 10.0)
x40 = (-22.0, -2.0)
x41 = (0.0, 20.0)
x42 = (5.0, 25.0)
x43 = (-8.0, 12.0)
x44 = (-28.0, -8.0)
x45 = (-28.0, -8.0)
x46 = (-10.0, 10.0)
x47 = (-25.0, -5.0)
x48 = (-9.0, 11.0)
x49 = (-15.0, 5.0)
x50 = (-7.0, 13.0)
x51 = (5.0, 25.0)
x52 = (-10.0, 10.0)
x53 = (-10.0, 10.0)
x54 = (8.0, 28.0)
x55 = (5.0, 25.0)
x56 = (-28.0, -8.0)
x57 = (-12.0, 8.0)
x58 = (-60.0, -40.0)
x59 = (26.0, 46.0)
x60 = (17.5, 37.5)
x61 = (-8.0, 12.0)
x62 = (-28.0, -8.0)
x63 = (-28.0, -8.0)
x64 = (-10.0, 10.0)
x65 = (-26.0, -6.0)
x66 = (-9.0, 11.0)
x67 = (-25.0, -5.0)
x68 = (3.0, 23.0)
x69 = (7.0, 27.0)
x70 = (-10.0, 10.0)
x71 = (-10.0, 10.0)
x72 = (8.0, 28.0)
x73 = (10.0, 30.0)
x74 = (-25.0, -5.0)
x75 = (-37.0, -17.0)
x76 = (-71.0, -51.0)
x77 = (38.0, 58.0)
x78 = (17.6, 37.6)
x79 = (5.0, 25.0)
x80 = (40.0, 60.0)
x81 = (-28.0, -8.0)
x82 = (-10.0, 10.0)
x83 = (-26.0, -6.0)
x84 = (7.0, 27.0)
x85 = (-3.0, 17.0)
x86 = (30.0, 50.0)
x87 = (6.0, 26.0)
x88 = (-25.0, -5.0)
x89 = (-10.0, 10.0)
x90 = (8.0, 28.0)
x91 = (0.0, 20.0)
x92 = (-27.0, -7.0)
x93 = (-46.0, -26.0)
x94 = (-22.0, -2.0)
x95 = (56.0, 76.0)
x96 = (8.0, 28.0)
x97 = (-5.0, 15.0)
x98 = (30.0, 50.0)
x99 = (-28.0, -8.0)
x100 = (-10.0, 10.0)
x101 = (-26.0, -6.0)
x102 = (7.0, 27.0)
x103 = (-3.0, 17.0)
x104 = (30.0, 50.0)
x105 = (4.0, 24.0)
x106 = (-15.0, 5.0)
x107 = (-10.0, 10.0)
x108 = (8.0, 28.0)
x109 = (0.0, 20.0)
x110 = (-27.0, -7.0)
x111 = (0.0, 20.0)
x112 = (20.0, 40.0)
x113 = (62.0, 82.0)
x114 = (26.200000000000003, 46.2)
x115 = (-25.0, -5.0)
x116 = (30.0, 50.0)
x117 = (-28.0, -8.0)
x118 = (-10.0, 10.0)
x119 = (-26.0, -6.0)
x120 = (-50.0, -30.0)
x121 = (-60.0, -40.0)
x122 = (-30.0, -10.0)
x123 = (4.5, 24.5)
x124 = (0.0, 20.0)
x125 = (-10.0, 10.0)
x126 = (8.0, 28.0)
x127 = (38.0, 58.0)
x128 = (-28.0, -8.0)
x129 = (-9.0, 11.0)
x130 = (14.0, 34.0)
x131 = (68.0, 88.0)
x132 = (15.0, 35.0)
x133 = (-10.0, 10.0)
x134 = (-40.0, -20.0)
x135 = (-21.0, -1.0)
x136 = (-10.0, 10.0)
x137 = (-26.0, -6.0)
x138 = (-9.0, 11.0)
x139 = (-25.0, -5.0)
x140 = (3.0, 23.0)
x141 = (4.5, 24.5)
x142 = (-10.0, 10.0)
x143 = (-28.0, -8.0)
x144 = (-2.0, 18.0)
x145 = (0.0, 20.0)
x146 = (-28.0, -8.0)
x147 = (-15.0, 5.0)
x148 = (-18.5, 1.5)
x149 = (-2.0, 18.0)
x150 = (15.0, 35.0)
x151 = (-10.0, 10.0)
x152 = (-28.0, -8.0)
x153 = (-18.0, 2.0)
x154 = (-10.0, 10.0)
x155 = (-19.0, 1.0)
x156 = (-9.0, 11.0)
x157 = (-15.0, 5.0)
x158 = (-7.0, 13.0)
x159 = (2.5, 22.5)
x160 = (-10.0, 10.0)
x161 = (-28.0, -8.0)
x162 = (-2.0, 18.0)
x163 = (0.0, 20.0)
x164 = (-15.0, 5.0)
x165 = (-10.0, 10.0)
x166 = (-15.0, 5.0)
x167 = (-7.0, 13.0)
x168 = (-7.0, 13.0)
x169 = (-10.0, 10.0)
x170 = (-28.0, -8.0)
x171 = (-18.0, 2.0)
x172 = (-10.0, 10.0)
x173 = (-5.0, 15.0)
x174 = (-10.0, 10.0)
x175 = (-15.0, 5.0)
x176 = (-7.0, 13.0)
x177 = (-13.0, 7.0)
x178 = (-10.0, 10.0)
x179 = (-28.0, -8.0)
x180 = (-2.0, 18.0)
x181 = (15.0, 35.0)
x182 = (-15.0, 5.0)
x183 = (-10.0, 10.0)
x184 = (-26.0, -6.0)
x185 = (11.0, 31.0)
x186 = (-8.5, 11.5)
x187 = (-10.0, 10.0)
x188 = (-28.0, -8.0)
x189 = (-18.0, 2.0)
x190 = (30.0, 50.0)
x191 = (-5.0, 15.0)
x192 = (-10.0, 10.0)
x193 = (-26.0, -6.0)
x194 = (11.0, 31.0)
x195 = (-11.5, 8.5)
x196 = (-10.0, 10.0)
x197 = (-28.0, -8.0)
x198 = (-2.0, 18.0)
x199 = (-5.0, 15.0)
x200 = (-15.0, 5.0)
x201 = (-10.0, 10.0)
x202 = (-26.0, -6.0)
x203 = (11.0, 31.0)
x204 = (-8.5, 11.5)
x205 = (-10.0, 10.0)
x206 = (-28.0, -8.0)
x207 = (-18.0, 2.0)
x208 = (30.0, 50.0)
x209 = (-5.0, 15.0)
x210 = (-10.0, 10.0)
x211 = (-26.0, -6.0)
x212 = (11.0, 31.0)
x213 = (-11.5, 8.5)
x214 = (-10.0, 10.0)
x215 = (-28.0, -8.0)
x216 = (-2.0, 18.0)

# 探索空間の設定 この場合は4つの空間を探索している
x = (x1, 
x2, 
x3, 
x4, 
x5, 
x6, 
x7, 
x8, 
x9, 
x10, 
x11, 
x12, 
x13, 
x14, 
x15, 
x16, 
x17, 
x18, 
x19, 
x20, 
x21, 
x22, 
x23, 
x24, 
x25, 
x26, 
x27, 
x28, 
x29, 
x30, 
x31, 
x32, 
x33, 
x34, 
x35, 
x36, 
x37, 
x38, 
x39, 
x40, 
x41, 
x42, 
x43, 
x44, 
x45, 
x46, 
x47, 
x48, 
x49, 
x50, 
x51, 
x52, 
x53, 
x54, 
x55, 
x56, 
x57, 
x58, 
x59, 
x60, 
x61, 
x62, 
x63, 
x64, 
x65, 
x66, 
x67, 
x68, 
x69, 
x70, 
x71, 
x72, 
x73, 
x74, 
x75, 
x76, 
x77, 
x78, 
x79, 
x80, 
x81, 
x82, 
x83, 
x84, 
x85, 
x86, 
x87, 
x88, 
x89, 
x90, 
x91, 
x92, 
x93, 
x94, 
x95, 
x96, 
x97, 
x98, 
x99, 
x100, 
x101, 
x102, 
x103, 
x104, 
x105, 
x106, 
x107, 
x108, 
x109, 
x110, 
x111, 
x112, 
x113, 
x114, 
x115, 
x116, 
x117, 
x118, 
x119, 
x120, 
x121, 
x122, 
x123, 
x124, 
x125, 
x126, 
x127, 
x128, 
x129, 
x130, 
x131, 
x132, 
x133, 
x134, 
x135, 
x136, 
x137, 
x138, 
x139, 
x140, 
x141, 
x142, 
x143, 
x144, 
x145, 
x146, 
x147, 
x148, 
x149, 
x150, 
x151, 
x152, 
x153, 
x154, 
x155, 
x156, 
x157, 
x158, 
x159, 
x160, 
x161, 
x162, 
x163, 
x164, 
x165, 
x166, 
x167, 
x168, 
x169, 
x170, 
x171, 
x172, 
x173, 
x174, 
x175, 
x176, 
x177, 
x178, 
x179, 
x180, 
x181, 
x182, 
x183, 
x184, 
x185, 
x186, 
x187, 
x188, 
x189, 
x190, 
x191, 
x192, 
x193, 
x194, 
x195, 
x196, 
x197, 
x198, 
x199, 
x200, 
x201, 
x202, 
x203, 
x204, 
x205, 
x206, 
x207, 
x208, 
x209, 
x210, 
x211, 
x212, 
x213, 
x214, 
x215, 
x216, 
)


result = gp_minimize(func, x, n_calls=100, noise=0.0,model_queue_size=1, verbose=True)
# n_call = サンプリング回数(試行回数) ここの数値分funcの処理を繰り返す。
# verbose = 探索時の標準出力を切り替える
