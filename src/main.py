import cv2
import numpy as np
import time
import ttc
import arrow
import command

prev_time = time.time()
prev_time2 = time.time()
frame_buffer = []

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
TTC_THRESHOLD = 0.1 #比較的厳しめ

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

        #TTC計算
        ttc_value = ttc.analysis(frame)
        if ttc_value is not None:
            frame_buffer.append(ttc_value)

        prev_time = current_time

    if elapsed_time2 >= 0.5: # 0.5秒ごとに処理
        
        #矢印判定
        arrow.Arrow().analysis(frame)

        prev_time2 = current_time

    # TTCの平均を計算して警告を出す
    if len(frame_buffer) == 5:
        average_ttc = np.mean(frame_buffer)
        print(average_ttc)
        if average_ttc < TTC_THRESHOLD:
            print("oh! Object is very close.")
            #数回でstopコマンド送信
        frame_buffer = []
    
    # 'q'キーで終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()