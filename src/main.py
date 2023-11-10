import cv2
import numpy as np
import time
import ttc
import arrow
import command
import threading
import queue

debug = False

prev_time = time.time()
prev_time2 = time.time()

frame_buffer = []

close_count = 0
is_avoid = False

arrow_count = 0
old_direction = ""
is_turn = False

is_moving = False
is_exit = False

frame_queue = queue.Queue()
g_frame = None

# TCPクライアントのセットアップ
command = command.Command()
command.connect('127.0.0.1', 8989, 1024, debug)

# TTC用パラメータの初期化
ttc = ttc.TTC()

# TTCの警告閾値
TTC_THRESHOLD = 0.08

def show_arrow_info(frame, text):
    """
    cv2の画面内に矢印の情報を表示します
    """
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

def takeoff():
    """
    takeoffコマンドを送信
    """
    command.send("takeoff")

def land():
    """
    landコマンドを送信
    """
    command.send("land")

def rc(erlon: str, elevator: str, srotol: str, lador: str):
    """
    rcコマンドを送信
    erlon: 正で右移動、負で左移動
    elevator: 正で前進、負で後進
    srotol: 正で上昇、負で下降
    lador: 正で右旋回、負で左旋回
    """
    command.send(f"rc {erlon} {elevator} {srotol} {lador}")

def cw(degree: str):
    """
    cwコマンドを送信
    時計回りに指定した角度分旋回する
    """
    command.send(f"cw {degree}")

def ccw(degree: str):
    """
    ccwコマンドを送信
    反時計回りに指定した角度分旋回する
    """
    command.send(f"ccw {degree}")

def move_drone():
    """
    ドローンの制御関数
    """
    global is_moving, is_turn, old_direction, is_avoid, is_exit

    while True:
        if is_exit:
            rc("0", "0", "0", "0")
            break

        if not is_moving:
            is_moving = True
            # takeoff()

        if is_moving & is_turn:
            if(old_direction == "Left"):
                # rc("0", "0", "0", "0")
                # ccw("90")
                is_turn = False

            elif(old_direction == "Right"):
                # rc("0", "0", "0", "0")
                # cw("90")
                is_turn = False

        if is_moving & is_avoid:
            #land()
            is_avoid = False

        # if is_moving & (not is_avoid) & (not is_turn):
            # rc("0", "10", "0", "0")

        time.sleep(0.1)

dorone_thread = threading.Thread(target=move_drone)
dorone_thread.start()

def calc_ttc():
    """
    ttc計算処理
    """
    global g_frame, prev_time, frame_buffer, close_count, is_avoid, is_turn, is_exit
    
    while not is_exit:
        if g_frame is None:
            continue

        current_time = time.time()
        elapsed_time = current_time - prev_time

        if elapsed_time >= 0.1:  # 0.1秒ごとに処理

            # TTC計算
            ttc_value = ttc.analysis(g_frame)
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
                # if((not is_avoid) & (not is_turn)):
                    # 5連続以上で接近判定になったら停止させ回避行動を開始
                    # if(close_count >= 5):
                        # is_avoid = True
            else:
                close_count = 0

            frame_buffer = []

ttc_thread = threading.Thread(target=calc_ttc)
ttc_thread.start()

def calc_arrow():
    """
    矢印判定処理
    """
    global g_frame, arrow_count, old_direction, is_turn, is_avoid, is_exit, prev_time2, frame_queue

    while not is_exit:
        if g_frame is None:
            continue

        # フレーム間の経過時間を計算
        current_time = time.time()
        elapsed_time2 = current_time  - prev_time2

        # 矢印判定処理
        if elapsed_time2 >= 0.1: # 0.1秒ごとに処理
        
            # 矢印判定とカメラ映像取得
            new_frame, direction, relative, position = arrow.Arrow().analysis(g_frame)
            if((direction is not None) & (relative is not None) & (position is not None)):
                lr, ud, dx, dy = position
                show_arrow_info(new_frame, f"Arrow:{direction}, X:{dx}({lr}), Y:{dy}({ud}), Z:{relative}")

                # 矢印が近すぎる場合は後進
                _  = "todo"

                # 一定の距離以内に近づかないと数えない
                if(relative >= 40000):
                    if(old_direction == direction): arrow_count += 1
                    old_direction = direction

                # 20回連続で方向検知で方向転換
                if((not is_avoid) & (not is_turn)):
                    if(arrow_count >= 20):
                        is_turn = True

            else:
                arrow_count = 0
                old_direction = ""

            frame_queue.put(new_frame)

            prev_time2 = current_time

arrow_thread = threading.Thread(target=calc_arrow)
arrow_thread.start()

def camera_thread():
    """
    カメラ映像を取得する
    """
    global g_frame

    # カメラストリームのセットアップ
    cap = cv2.VideoCapture('udp://@0.0.0.0:11113')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)
    while True:
        if is_exit:
            break

        _, frame = cap.read()
        
        # 無効なフレームはスキップ
        if frame is None or frame.size == 0:
            continue

        g_frame = frame

    cap.release()


video_thread = threading.Thread(target=camera_thread)
video_thread.start()

while True:
    frame = frame_queue.get()
    cv2.imshow("Tello-Detection", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        is_exit = True
        break

cv2.destroyAllWindows()
arrow_thread.join()
ttc_thread.join()
dorone_thread.join()
video_thread.join()
