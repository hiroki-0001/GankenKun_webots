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
const.motor_parameter = 19 #首のヨー軸は関係なさそうなので、抜きました

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(
    -1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
children.importMFNodeFromString(
    -1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0 0.450 rotation 0 0 1 0 controller "play_motion"}}')
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
            print("### 現在の飛距離 = {} ###".format(pos[0]))
            with open('result.csv', 'a', newline = '') as f:
                writer = csv.writer(f)
                writer.writerow([pos[0], pos[1]])
    return -pos[0]

    # 各パラメータの探索範囲
#各フレームあたりの時間の探索範囲
x0 = (99.0, 101.0)
x1 = (-6.0, -4.0)
x2 = (-1.0, 1.0)
x3 = (-17.0, -15.0)
x4 = (20.0, 22.0)
x5 = (0.5, 2.5)
x6 = (-1.0, 1.0)
x7 = (-19.0, -17.0)
x8 = (-9.0, -7.0)
x9 = (39.0, 41.0)
x10 = (4.0, 6.0)
x11 = (-1.0, 1.0)
x12 = (-17.0, -15.0)
x13 = (20.0, 22.0)
x14 = (-2.5, -0.5)
x15 = (-1.0, 1.0)
x16 = (-19.0, -17.0)
x17 = (7.0, 9.0)
x18 = (39.0, 41.0)
x19 = (14.0, 16.0)
x20 = (-15.0, -13.0)
x21 = (-1.0, 1.0)
x22 = (-23.0, -21.0)
x23 = (21.0, 23.0)
x24 = (13.0, 15.0)
x25 = (1.0, 3.0)
x26 = (-19.0, -17.0)
x27 = (-19.0, -17.0)
x28 = (-1.0, 1.0)
x29 = (-11.0, -9.0)
x30 = (-2.0, 0.0)
x31 = (-21.0, -19.0)
x32 = (19.0, 21.0)
x33 = (13.0, 15.0)
x34 = (-1.0, 1.0)
x35 = (-1.0, 1.0)
x36 = (17.0, 19.0)
x37 = (-1.0, 1.0)
x38 = (9.0, 11.0)
x39 = (-15.0, -13.0)
x40 = (-1.0, 1.0)
x41 = (-13.0, -11.0)
x42 = (9.0, 11.0)
x43 = (14.0, 16.0)
x44 = (1.0, 3.0)
x45 = (-19.0, -17.0)
x46 = (-19.0, -17.0)
x47 = (-1.0, 1.0)
x48 = (-16.0, -14.0)
x49 = (0.0, 2.0)
x50 = (-6.0, -4.0)
x51 = (2.0, 4.0)
x52 = (14.0, 16.0)
x53 = (-1.0, 1.0)
x54 = (-1.0, 1.0)
x55 = (17.0, 19.0)
x56 = (-1.0, 1.0)
x57 = (14.0, 16.0)
x58 = (-19.0, -17.0)
x59 = (-3.0, -1.0)
x60 = (-51.0, -49.0)
x61 = (35.0, 37.0)
x62 = (26.5, 28.5)
x63 = (1.0, 3.0)
x64 = (-19.0, -17.0)
x65 = (-19.0, -17.0)
x66 = (-1.0, 1.0)
x67 = (-17.0, -15.0)
x68 = (0.0, 2.0)
x69 = (-16.0, -14.0)
x70 = (12.0, 14.0)
x71 = (16.0, 18.0)
x72 = (-1.0, 1.0)
x73 = (-1.0, 1.0)
x74 = (17.0, 19.0)
x75 = (-1.0, 1.0)
x76 = (19.0, 21.0)
x77 = (-16.0, -14.0)
x78 = (-28.0, -26.0)
x79 = (-62.0, -60.0)
x80 = (47.0, 49.0)
x81 = (26.6, 28.6)
x82 = (14.0, 16.0)
x83 = (49.0, 51.0)
x84 = (-19.0, -17.0)
x85 = (-1.0, 1.0)
x86 = (-17.0, -15.0)
x87 = (16.0, 18.0)
x88 = (6.0, 8.0)
x89 = (39.0, 41.0)
x90 = (15.0, 17.0)
x91 = (-16.0, -14.0)
x92 = (-1.0, 1.0)
x93 = (17.0, 19.0)
x94 = (-1.0, 1.0)
x95 = (9.0, 11.0)
x96 = (-18.0, -16.0)
x97 = (-37.0, -35.0)
x98 = (-13.0, -11.0)
x99 = (65.0, 67.0)
x100 = (17.0, 19.0)
x101 = (4.0, 6.0)
x102 = (39.0, 41.0)
x103 = (-19.0, -17.0)
x104 = (-1.0, 1.0)
x105 = (-17.0, -15.0)
x106 = (16.0, 18.0)
x107 = (6.0, 8.0)
x108 = (39.0, 41.0)
x109 = (13.0, 15.0)
x110 = (-6.0, -4.0)
x111 = (-1.0, 1.0)
x112 = (17.0, 19.0)
x113 = (-1.0, 1.0)
x114 = (9.0, 11.0)
x115 = (-18.0, -16.0)
x116 = (9.0, 11.0)
x117 = (29.0, 31.0)
x118 = (71.0, 73.0)
x119 = (35.2, 37.2)
x120 = (-16.0, -14.0)
x121 = (39.0, 41.0)
x122 = (-19.0, -17.0)
x123 = (-1.0, 1.0)
x124 = (-17.0, -15.0)
x125 = (-41.0, -39.0)
x126 = (-51.0, -49.0)
x127 = (-21.0, -19.0)
x128 = (13.5, 15.5)
x129 = (9.0, 11.0)
x130 = (-1.0, 1.0)
x131 = (17.0, 19.0)
x132 = (-1.0, 1.0)
x133 = (47.0, 49.0)
x134 = (-19.0, -17.0)
x135 = (0.0, 2.0)
x136 = (23.0, 25.0)
x137 = (77.0, 79.0)
x138 = (24.0, 26.0)
x139 = (-1.0, 1.0)
x140 = (-31.0, -29.0)
x141 = (-12.0, -10.0)
x142 = (-1.0, 1.0)
x143 = (-17.0, -15.0)
x144 = (0.0, 2.0)
x145 = (-16.0, -14.0)
x146 = (12.0, 14.0)
x147 = (13.5, 15.5)
x148 = (-1.0, 1.0)
x149 = (-19.0, -17.0)
x150 = (7.0, 9.0)
x151 = (-1.0, 1.0)
x152 = (9.0, 11.0)
x153 = (-19.0, -17.0)
x154 = (-6.0, -4.0)
x155 = (-9.5, -7.5)
x156 = (7.0, 9.0)
x157 = (24.0, 26.0)
x158 = (-1.0, 1.0)
x159 = (-19.0, -17.0)
x160 = (-9.0, -7.0)
x161 = (-1.0, 1.0)
x162 = (-10.0, -8.0)
x163 = (0.0, 2.0)
x164 = (-6.0, -4.0)
x165 = (2.0, 4.0)
x166 = (11.5, 13.5)
x167 = (-1.0, 1.0)
x168 = (-19.0, -17.0)
x169 = (7.0, 9.0)
x170 = (-1.0, 1.0)
x171 = (9.0, 11.0)
x172 = (-6.0, -4.0)
x173 = (-1.0, 1.0)
x174 = (-6.0, -4.0)
x175 = (2.0, 4.0)
x176 = (2.0, 4.0)
x177 = (-1.0, 1.0)
x178 = (-19.0, -17.0)
x179 = (-9.0, -7.0)
x180 = (-1.0, 1.0)
x181 = (4.0, 6.0)
x182 = (-1.0, 1.0)
x183 = (-6.0, -4.0)
x184 = (2.0, 4.0)
x185 = (-4.0, -2.0)
x186 = (-1.0, 1.0)
x187 = (-19.0, -17.0)
x188 = (7.0, 9.0)
x189 = (-1.0, 1.0)
x190 = (24.0, 26.0)
x191 = (-6.0, -4.0)
x192 = (-1.0, 1.0)
x193 = (-17.0, -15.0)
x194 = (20.0, 22.0)
x195 = (0.5, 2.5)
x196 = (-1.0, 1.0)
x197 = (-19.0, -17.0)
x198 = (-9.0, -7.0)
x199 = (39.0, 41.0)
x200 = (4.0, 6.0)
x201 = (-1.0, 1.0)
x202 = (-17.0, -15.0)
x203 = (20.0, 22.0)
x204 = (-2.5, -0.5)
x205 = (-1.0, 1.0)
x206 = (-19.0, -17.0)
x207 = (7.0, 9.0)
x208 = (39.0, 41.0)
x209 = (4.0, 6.0)
x210 = (-6.0, -4.0)
x211 = (-1.0, 1.0)
x212 = (-17.0, -15.0)
x213 = (20.0, 22.0)
x214 = (0.5, 2.5)
x215 = (-1.0, 1.0)
x216 = (-19.0, -17.0)
x217 = (-9.0, -7.0)
x218 = (39.0, 41.0)
x219 = (4.0, 6.0)
x220 = (-1.0, 1.0)
x221 = (-17.0, -15.0)
x222 = (20.0, 22.0)
x223 = (-2.5, -0.5)
x224 = (-1.0, 1.0)
x225 = (-19.0, -17.0)
x226 = (7.0, 9.0)
x227 = (39.0, 41.0)

# 探索空間の設定 この場合は4つの空間を探索している
x = (x0, 
x1, 
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
x217, 
x218, 
x219, 
x220, 
x221, 
x222, 
x223, 
x224, 
x225, 
x226, 
x227, 
)



result = gp_minimize(func, x, n_calls=100, noise=0.0,model_queue_size=1, verbose=True)
# n_call = サンプリング回数(試行回数) ここの数値分funcの処理を繰り返す。
# verbose = 探索時の標準出力を切り替える
