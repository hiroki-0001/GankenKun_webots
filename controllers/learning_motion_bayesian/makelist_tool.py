import csv 

def get_motiondata():
    with open("../GankenKun_webots/controllers/play_motion/before_optimize_kick.csv", "r") as f:  # 最適化処理前のモーションファイルを読み込む
        # delimiter = 区切り文字の指定, lineterminator = 改行文字の指定
        read_data = csv.reader(f, delimiter=",", lineterminator="\r\n")
        # リスト内包表記を使用し、read_dataからデータを二次元配列として取得
        data = [row for row in read_data]
        return data

motion_data = get_motiondata()
count = 1

for frame in range(12):
    for servo_num in range(18):
        data = motion_data[frame][servo_num]
        print(" x{} = ({}, {})".format(count, float(data) - 10, float(data) + 10))
        count += 1

print("=====================================")

print(count)

print("x = (", end = '')
for i in range (count):
    print("x{}, ".format(i))

print(")")