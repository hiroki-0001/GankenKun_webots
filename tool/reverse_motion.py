import csv 

def get_motiondata():
    with open("kick.csv", "r") as f:  # 最適化処理前のモーションファイルを読み込む
        # delimiter = 区切り文字の指定, lineterminator = 改行文字の指定
        read_data = csv.reader(f, delimiter=",", lineterminator="\r\n")
        # リスト内包表記を使用し、read_dataからデータを二次元配列として取得
        data = [row for row in read_data]
        return data

def file_update(motiondata):
    with open("./reverse_kick.csv", "w") as f:  # motion.csvに上書きする(前の状態から更新される)
        writer = csv.writer(f)  # motion.csvに書き込む準備
        writer.writerows(motiondata)  # 1行ずつ書き込み
        print("file write")


def main():
    motion_data = get_motiondata()
    

    for frame in range(13):
        motion_data[frame][1], motion_data[frame][10] = motion_data[frame][10], motion_data[frame][1] # 足首ロール
        motion_data[frame][2], motion_data[frame][11] = motion_data[frame][11], motion_data[frame][2] # 足首ピッチ
        motion_data[frame][3], motion_data[frame][12] = motion_data[frame][12], motion_data[frame][3] # 膝ピッチ
        motion_data[frame][4], motion_data[frame][13] = motion_data[frame][13], motion_data[frame][4] # 脚付け根ピッチ
        motion_data[frame][5], motion_data[frame][14] = motion_data[frame][14], motion_data[frame][5] # 脚付け根ロール
        motion_data[frame][6], motion_data[frame][15] = motion_data[frame][15], motion_data[frame][6] # 脚付け根ヨー
        motion_data[frame][7], motion_data[frame][16] = motion_data[frame][16], motion_data[frame][7] # 肩ピッチ
        motion_data[frame][8], motion_data[frame][17] = motion_data[frame][17], motion_data[frame][8] # 肩ロール
        motion_data[frame][9], motion_data[frame][18] = motion_data[frame][18], motion_data[frame][9] # 肘ピッチ
    
    for frame in range(13):
        for servo_num in range(19):
            if(servo_num == 1 or servo_num == 10 or servo_num == 5 or servo_num == 14 or servo_num == 6 or servo_num == 15 or servo_num == 8 or servo_num == 17):
                tmp = float(motion_data[frame][servo_num])
                tmp = tmp * -1
                motion_data[frame][servo_num] = tmp
    

    file_update(motion_data)


if __name__ == "__main__":
    main()