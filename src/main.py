import cv2
import numpy as np
import time
import ttc
import arrow
import command

prev_time = time.time()
prev_time2 = time.time()

frame_buffer = []

close_count = 0
is_avoid = False

arrow_count = 0
old_direction = ""
is_turn = False

# TCPクライアントのセットアップ
command = command.Command()
command.connect('127.0.0.1', 8989, 1024)

# カメラストリームのセットアップ
cap = cv2.VideoCapture('udp://@0.0.0.0:11113')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)

# TTC用パラメータの初期化
ttc = ttc.TTC()

# TTCの警告閾値
TTC_THRESHOLD = 0.1 # 比較的厳しめ

def show_arrow_info(frame, text):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    thickness = 2
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]

    # テキストの背景の位置を設定
    text_offset_x = 10
    text_offset_y = 50
    box_coords = (
        (text_offset_x, text_offset_y + 5),
        (text_offset_x + text_size[0] + 5, text_offset_y - text_size[1] - 5)
    )
    cv2.rectangle(frame, box_coords[0], box_coords[1], (255, 255, 255), cv2.FILLED)

    # フレームにテキストを描画
    cv2.putText(frame, text, (text_offset_x, text_offset_y), font, font_scale, (0, 0, 0), thickness)

while True:
    # ビデオフレームの読み込み
    ret, frame = cap.read()

    # 無効なフレームはスキップ
    if frame is None or frame.size == 0:
        continue

    # フレーム間の経過時間を計算
    current_time = time.time()
    elapsed_time = current_time - prev_time
    elapsed_time2 = current_time  - prev_time2

    if elapsed_time >= 0.1:  # 0.1秒ごとに処理

        # TTC計算
        ttc_value = ttc.analysis(frame)
        if ttc_value is not None:
            frame_buffer.append(ttc_value)

        prev_time = current_time

    # TTCの平均を計算して警告を出す
    if len(frame_buffer) == 5:
        average_ttc = np.mean(frame_buffer)
        print(f"TTC: {average_ttc}")

        # 至近距離判定カウント
        if average_ttc < TTC_THRESHOLD:
            print("TTC: Object is very close.")
            close_count += 1
            if((not is_avoid) & (not is_turn)):
                # 5連続以上で接近判定になったら停止させ回避行動を開始
                if(close_count >= 5):
                    is_avoid = True
                    command.send("ABCDEFGHIJKLNMOPQRSTUVWXYZ1234567890") # todo
        else:
            close_count = 0

        if is_avoid:
            # 一定時間(回避処理終了)後再検査 (okのレスポンスでも良い？)
            _ = "todo"

        frame_buffer = []

    # 矢印判定処理
    if elapsed_time2 >= 0.25: # 0.25秒ごとに処理
    
        # 矢印判定とカメラ映像取得
        new_frame, direction, relative, position = arrow.Arrow().analysis(frame)
        if((direction is not None) & (relative is not None) & (position is not None)):
            lr, ud, dx, dy = position
            show_arrow_info(new_frame, f"Arrow:{direction}, X:{dx}({lr}), Y:{dy}({ud}), Z:{relative}")

            # 一定の距離以内に近づかないと数えない
            if(relative >= 100000):
                if(old_direction == direction): arrow_count += 1
                old_direction = direction

            # 5回連続で方向検知で方向転換
            if((not is_avoid) & (not is_turn)):
                if(arrow_count >= 5):
                    is_turn = True
                    command.send("АБВГДЕЁЖЗИЙКЛМНОПРСТУФХЦЧШЩЪЫЬЭЮЯ") # todo

            elif is_turn:
                # 位置調整など 指示ごとに一定時間経過後処理 (okレスポンスでも良い？)
                _ = "todo"

        else:
            arrow_count = 0
            old_direction = ""

        # 出力
        cv2.namedWindow('Tello-Detection', cv2.WINDOW_NORMAL)
        cv2.imshow("Tello-Detection", new_frame)

        prev_time2 = current_time
    
    
    # 'Esc'キーで終了
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()