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

def file_read():
    with open("../play_motion/kick_motion.csv", "r") as f:  # kick_motion.csvを読み込む
        # delimiter = 区切り文字の指定, lineterminator = 改行文字の指定
        read_data = csv.reader(f, delimiter=",", lineterminator="\r\n")
        # リスト内包表記を使用し、read_dataからデータを二次元配列として取得
        data = [row for row in read_data]
        return data

def select_parameter(param, motiondata):
    motiondata[6][0] = str(int(param[0])) #モーションファイルの6行0列目 = 6フレーム目の時間を指定している
    motiondata[7][0] = str(int(param[1])) #モーションファイルの7行0列目 = 7フレーム目の時間を指定している
    motiondata[7][3] = str(int(param[2])) #モーションファイルの7行3列目 = 足先ピッチの角度を指定している
    motiondata[7][4] = str(int(param[3])) #モーションファイルの7行4列目 = 膝ピッチの角度を指定している
    return param

def file_update(motiondata):
    with open("../play_motion/motion.csv", "w") as f:  # motion.csvに上書きする(前の状態から更新される)
        writer = csv.writer(f)  # motion.csvに書き込む準備
        writer.writerows(motiondata)  # 1行ずつ書き込み
        print("file write")


def func(param):  # 最適化関数の設定
    global player, ball, children, ball_translation, ball_rotation, supervisor  # グローバル変数の定義

# ファイルの書き換えの処理
    motiondata = file_read()
    param = select_parameter(param, motiondata)
    file_update(motiondata)


# ----------------
#
    count = 0
    player.remove()
    print("robot clean")
    # GanekenKunの呼び出し 一度消して再生成することで新たなmotionファイルを対応させたロボットを呼び出す
    # ここでGankenKunの位置座標の指定も行っている。
    children.importMFNodeFromString(-1, f'DEF PLAYER RoboCup_GankenKun {{translation -0.2 0.1 0.450 rotation 0 0 1 0 controller "play_motion" controllerArgs "motion.csv"}}')
    # playerにsupervisorを与えている.(これをしないとsupervisorの.removeが使用できなくなる)
    player = supervisor.getFromDef('PLAYER')
    # ボール物理情報を初期化
    ball.resetPhysics()
    # ボールの位置を所定の位置に設置する(フィールドの真ん中に置く)
    # [x, y, z] 単位は[m]
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
                writer.writerow([param[0], param[1], param[2],param[3], pos[0], pos[1]])
    return -pos[0]



    # 各パラメータの探索範囲
x0 = (1, 50)
x1 = (1, 50)
x2 = (0, 80)
x3 = (0, 80)
# 探索空間の設定 この場合は4つの空間を探索している
x = (x0, x1, x2, x3)
result = gp_minimize(func, x, n_calls=100, noise=0.0,model_queue_size=1, verbose=True)
# n_call = サンプリング回数(試行回数) ここの数値分funcの処理を繰り返す。
# verbose = 探索時の標準出力を切り替える
