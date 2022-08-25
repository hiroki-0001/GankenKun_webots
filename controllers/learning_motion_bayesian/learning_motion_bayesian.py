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

const.frame_parameter = 11
const.motor_parameter = 19 #首のヨー軸は関係なさそうなので、抜きました
max_dist = 0

# start the webots supervisor
supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())
field = Field("kid")
children = supervisor.getRoot().getField('children')
children.importMFNodeFromString(-1, f'RobocupSoccerField {{ size "kid" }}')
children.importMFNodeFromString(-1, f'DEF BALL RobocupSoccerBall {{ translation 0 0 0.1 size 1 }}')
#右キック用
children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 0.1 0.450 rotation 0 0 1 0 controller "play_motion"}}')
#左キック用
#children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.3 -0.1 0.450 rotation 0 0 1 0 controller "play_motion"}}')

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

def file_clean():
    with open("./result.csv", 'w') as f:
        pass

def after_optimize_file_update(motiondata):
    with open("../play_motion/after_optimize_motion.csv", "w") as f:  # motion.csvに上書きする(前の状態から更新される)
        writer = csv.writer(f)  # motion.csvに書き込む準備
        writer.writerows(motiondata)  # 1行ずつ書き込み
        print("file write")


def func(param):  # 最適化関数の設定
    global player, ball, children, ball_translation, ball_rotation, supervisor,max_dist  # グローバル変数の定義
    motiondata = get_motiondata()
    param = select_parameter(param, motiondata)
    file_update(motiondata)
    count = 0
    player.remove()
    #右キック用
    children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.2 0.1 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')
    #左キック用
    #children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.2 -0.1 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')

    player = supervisor.getFromDef('PLAYER')
    player_translation = supervisor.getFromDef('PLAYER').getField('translation')
    ball.resetPhysics()
    ball_translation.setSFVec3f([0, 0, 0.1])
    ball_rotation.setSFRotation([0, 0, 1, 0])
    while supervisor.step(time_step) != -1:
        count += 1
        if count > 800:
            break
        if count > 800 - 1:
            pos = ball_translation.getSFVec3f()
            player_pos = player_translation.getSFVec3f()
            print("player pos = {}".format(player_pos))
            if(player_pos[2] > 0.25):
                if max_dist < pos[0]:
                    max_dist = pos[0]
                    after_optimize_file_update(motiondata)
                    print("最高記録を更新しました")
            else:
                print("転倒しました")
                pos[0] = 0
            print("### 現在の飛距離 = {} ###".format(pos[0]))
            print("### 現在までの最高飛距離 = {} ###".format(max_dist))
            with open('result.csv', 'a', newline = '') as f:
                writer = csv.writer(f)
                writer.writerow([pos[0], pos[1]])
    return -pos[0]

    # 各パラメータの探索範囲
#各フレームあたりの時間の探索範囲
x0 = (45.0, 55.0)
x1 = (-19.0, -9.0)
x2 = (-5.0, 5.0)
x3 = (-27.0, -17.0)
x4 = (17.0, 27.0)
x5 = (9.0, 19.0)
x6 = (-3.0, 7.0)
x7 = (-23.0, -13.0)
x8 = (-23.0, -13.0)
x9 = (35.0, 45.0)
x10 = (-15.0, -5.0)
x11 = (-6.0, 4.0)
x12 = (-25.0, -15.0)
x13 = (15.0, 25.0)
x14 = (9.0, 19.0)
x15 = (-5.0, 5.0)
x16 = (-23.0, -13.0)
x17 = (3.0, 13.0)
x18 = (35.0, 45.0)
x19 = (35.0, 45.0)
x20 = (-19.0, -9.0)
x21 = (-5.0, 5.0)
x22 = (-17.0, -7.0)
x23 = (5.0, 15.0)
x24 = (10.0, 20.0)
x25 = (-3.0, 7.0)
x26 = (-23.0, -13.0)
x27 = (-23.0, -13.0)
x28 = (35.0, 45.0)
x29 = (-20.0, -10.0)
x30 = (-4.0, 6.0)
x31 = (-10.0, 0.0)
x32 = (-2.0, 8.0)
x33 = (10.0, 20.0)
x34 = (-5.0, 5.0)
x35 = (-23.0, -13.0)
x36 = (3.0, 13.0)
x37 = (35.0, 45.0)
x38 = (15.0, 25.0)
x39 = (-23.0, -13.0)
x40 = (-7.0, 3.0)
x41 = (-55.0, -45.0)
x42 = (31.0, 41.0)
x43 = (22.5, 32.5)
x44 = (-3.0, 7.0)
x45 = (-23.0, -13.0)
x46 = (-34.0, -24.0)
x47 = (35.0, 45.0)
x48 = (-23.0, -13.0)
x49 = (-4.0, 6.0)
x50 = (-20.0, -10.0)
x51 = (8.0, 18.0)
x52 = (12.0, 22.0)
x53 = (-5.0, 5.0)
x54 = (-23.0, -13.0)
x55 = (3.0, 13.0)
x56 = (35.0, 45.0)
x57 = (25.0, 35.0)
x58 = (-20.0, -10.0)
x59 = (-32.0, -22.0)
x60 = (-60.0, -50.0)
x61 = (33.0, 43.0)
x62 = (22.6, 32.6)
x63 = (9.0, 19.0)
x64 = (-23.0, -13.0)
x65 = (-33.0, -23.0)
x66 = (35.0, 45.0)
x67 = (-23.0, -13.0)
x68 = (-4.0, 6.0)
x69 = (-20.0, -10.0)
x70 = (8.0, 18.0)
x71 = (11.0, 21.0)
x72 = (-5.0, 5.0)
x73 = (-23.0, -13.0)
x74 = (3.0, 13.0)
x75 = (35.0, 45.0)
x76 = (1, 7.0)
x77 = (-19.0, -9.0)
x78 = (-33.0, -23.0)
x79 = (-12.0, -2.0)
x80 = (55.0, 65.0)
x81 = (15.0, 25.0)
x82 = (-5.0, 5.0)
x83 = (-23.0, -13.0)
x84 = (-23.0, -13.0)
x85 = (35.0, 45.0)
x86 = (-23.0, -13.0)
x87 = (-4.0, 6.0)
x88 = (-20.0, -10.0)
x89 = (8.0, 18.0)
x90 = (9.0, 19.0)
x91 = (-5.0, 5.0)
x92 = (-23.0, -13.0)
x93 = (3.0, 13.0)
x94 = (35.0, 45.0)
x95 = (1, 9.0)
x96 = (-18.0, -8.0)
x97 = (-25.0, -15.0)
x98 = (23.0, 33.0)
x99 = (67.0, 77.0)
x100 = (20.2, 30.2)
x101 = (-15.0, -5.0)
x102 = (-23.0, -13.0)
x103 = (-23.0, -13.0)
x104 = (35.0, 45.0)
x105 = (-23.0, -13.0)
x106 = (-11.0, -1.0)
x107 = (-27.0, -17.0)
x108 = (-5.0, 5.0)
x109 = (9.5, 19.5)
x110 = (-5.0, 5.0)
x111 = (-23.0, -13.0)
x112 = (3.0, 13.0)
x113 = (35.0, 45.0)
x114 = (1, 9.0)
x115 = (-18.0, -8.0)
x116 = (-5.0, 5.0)
x117 = (23.0, 33.0)
x118 = (87.0, 97.0)
x119 = (20.2, 30.2)
x120 = (-15.0, -5.0)
x121 = (-23.0, -13.0)
x122 = (-23.0, -13.0)
x123 = (35.0, 45.0)
x124 = (-21.0, -11.0)
x125 = (-11.0, -1.0)
x126 = (-27.0, -17.0)
x127 = (-5.0, 5.0)
x128 = (9.5, 19.5)
x129 = (-5.0, 5.0)
x130 = (-23.0, -13.0)
x131 = (3.0, 13.0)
x132 = (35.0, 45.0)
x133 = (1, 9.0)
x134 = (-18.0, -8.0)
x135 = (15.0, 25.0)
x136 = (23.0, 33.0)
x137 = (87.0, 97.0)
x138 = (20.2, 30.2)
x139 = (-15.0, -5.0)
x140 = (-23.0, -13.0)
x141 = (-23.0, -13.0)
x142 = (35.0, 45.0)
x143 = (-21.0, -11.0)
x144 = (-11.0, -1.0)
x145 = (-27.0, -17.0)
x146 = (-5.0, 5.0)
x147 = (9.5, 19.5)
x148 = (-5.0, 5.0)
x149 = (-23.0, -13.0)
x150 = (3.0, 13.0)
x151 = (35.0, 45.0)
x152 = (20.0, 30.0)
x153 = (-21.0, -11.0)
x154 = (-5.0, 5.0)
x155 = (-51.5, -41.5)
x156 = (41.0, 51.0)
x157 = (20.0, 30.0)
x158 = (-5.0, 5.0)
x159 = (-23.0, -13.0)
x160 = (-16.0, -6.0)
x161 = (35.0, 45.0)
x162 = (-23.0, -13.0)
x163 = (-6.0, 4.0)
x164 = (-10.0, 0.0)
x165 = (-2.0, 8.0)
x166 = (9.5, 19.5)
x167 = (-5.0, 5.0)
x168 = (-23.0, -13.0)
x169 = (3.0, 13.0)
x170 = (35.0, 45.0)
x171 = (5.0, 15.0)
x172 = (-23.0, -13.0)
x173 = (-5.0, 5.0)
x174 = (-20.5, -10.5)
x175 = (5.0, 15.0)
x176 = (20.0, 30.0)
x177 = (-5.0, 5.0)
x178 = (-23.0, -13.0)
x179 = (-13.0, -3.0)
x180 = (35.0, 45.0)
x181 = (-14.0, -4.0)
x182 = (-6.0, 4.0)
x183 = (-10.0, 0.0)
x184 = (-2.0, 8.0)
x185 = (7.5, 17.5)
x186 = (-5.0, 5.0)
x187 = (-23.0, -13.0)
x188 = (3.0, 13.0)
x189 = (35.0, 45.0)
x190 = (95.0, 105.0)
x191 = (-13.0, -3.0)
x192 = (-5.0, 5.0)
x193 = (-10.0, 0.0)
x194 = (-2.0, 8.0)
x195 = (-8.0, 2.0)
x196 = (-5.0, 5.0)
x197 = (-23.0, -13.0)
x198 = (-13.0, -3.0)
x199 = (35.0, 45.0)
x200 = (0.0, 10.0)
x201 = (-5.0, 5.0)
x202 = (-10.0, 0.0)
x203 = (-2.0, 8.0)
x204 = (-8.0, 2.0)
x205 = (-5.0, 5.0)
x206 = (-23.0, -13.0)
x207 = (3.0, 13.0)
x208 = (35.0, 45.0)


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
)

file_clean()
result = gp_minimize(func, x, n_calls=1000, noise=0.0,model_queue_size=1, verbose=True)
# n_call = サンプリング回数(試行回数) ここの数値分funcの処理を繰り返す。
# verbose = 探索時の標準出力を切り替える
