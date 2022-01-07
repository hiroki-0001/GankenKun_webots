import csv 

def get_motiondata():
    with open("../play_motion/before_optimize_kick.csv", "r") as f:  # 最適化処理前のモーションファイルを読み込む
        # delimiter = 区切り文字の指定, lineterminator = 改行文字の指定
        read_data = csv.reader(f, delimiter=",", lineterminator="\r\n")
        # リスト内包表記を使用し、read_dataからデータを二次元配列として取得
        data = [row for row in read_data]
        return data

def main():
    motion_data = get_motiondata()
    param_range = 10
    count = 1

    for frame in range(12):
        for servo_num in range(19):
            print("frame = {}, servo_num = {}".format(frame, servo_num))
            base_data = float(motion_data[frame][servo_num])
            min_data = base_data - param_range
            max_data = base_data + param_range

            if servo_num == 0 and min_data <= 0:
                print("x{} = (1, {})".format(count, max_data))
            else:    
                print("x{} = ({}, {})".format(count, min_data, max_data))
            count += 1

    print("=====================================")
    print(count)
    print("x = (", end = '')
    for i in range (count):
        print("x{}, ".format(i))
    print(")")

if __name__ == "__main__":
    main()