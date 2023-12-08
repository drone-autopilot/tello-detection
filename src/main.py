import cv2
import numpy as np
import time
import ttc
import arrow
import command
import threading
import queue
import socket
import json

#                                                                                                                                              
#                                                                                                 ■■             ■             ■               
#                                                                                          ■■    ■■          ■■■■■■  ■■■■■■■■■■■             ■ 
# ■■■■■■■■■■        ■■■■■■■■■■■■   ■■■■■■■■■■     ■■■■■■■   ■■■■■■        ■■■■■■     ■■■■■■■■ ■  ■■     ■                ■    ■■            ■■■
#   ■■■    ■■■        ■■■      ■     ■■■    ■■      ■■         ■        ■■■    ■■■      ■     ■■■■■■■■■■■■         ■    ■■    ■■            ■■■
#   ■■■      ■■       ■■■      ■     ■■■     ■■     ■■         ■      ■■■       ■■     ■■    ■■ ■■     ■■   ■■■■■■■■■ ■■■■    ■■            ■■■
#   ■■■       ■■      ■■■      ■     ■■■     ■■     ■■         ■      ■■        ■■     ■■    ■  ■■ ■■  ■                ■■■   ■■            ■■■
#   ■■■       ■■■     ■■■            ■■■     ■■     ■■         ■     ■■         ■■     ■        ■  ■                   ■■ ■■  ■■            ■■ 
#    ■■        ■■      ■■             ■■     ■■     ■■         ■     ■■                ■       ■■  ■   ■■    ■■■■■■    ■■  ■  ■■            ■■ 
#    ■■        ■■      ■■    ■        ■■     ■■     ■■         ■    ■■■                ■■■■■■  ■■■■■■■■■■             ■■      ■             ■■ 
#    ■■        ■■      ■■   ■■        ■■   ■■■      ■■         ■    ■■■               ■■   ■  ■■   ■                  ■    ■■■■             ■■ 
#    ■■        ■■      ■■■■■■■        ■■■■■■        ■■         ■    ■■■               ■■   ■ ■ ■   ■   ■     ■■■■■■  ■      ■■               ■ 
#    ■■        ■■      ■■   ■■        ■■    ■■■     ■■         ■    ■■■      ■■■■■■  ■■■   ■   ■■■■■■■■■■               ■■■                  ■ 
#    ■■        ■■      ■■    ■        ■■     ■■■    ■■         ■    ■■■         ■■   ■ ■   ■   ■   ■                      ■■                 ■ 
#    ■■        ■■      ■■             ■■      ■■    ■■         ■     ■■         ■■     ■   ■   ■   ■         ■■■■■■■   ■■  ■                 ■ 
#    ■■       ■■       ■■             ■■      ■■    ■■        ■■     ■■         ■■     ■   ■   ■   ■   ■     ■    ■  ■ ■■     ■              ■ 
#    ■■       ■■      ■■■       ■    ■■■      ■■     ■■       ■       ■■        ■■     ■   ■   ■■■■■■■■■■    ■    ■  ■ ■■      ■               
#   ■■■      ■■       ■■■      ■■    ■■■     ■■■     ■■■     ■■       ■■■       ■■     ■   ■   ■   ■         ■    ■  ■ ■■      ■■              
#   ■■■    ■■■        ■■■      ■■    ■■■    ■■■       ■■■■■■■■          ■■■    ■■■     ■■■■■   ■   ■         ■    ■ ■■ ■■      ■■           ■■ 
# ■■■■■■■■■■        ■■■■■■■■■■■■■  ■■■■■■■■■■          ■■■■■■            ■■■■■■■       ■   ■   ■   ■   ■■    ■■■■■■ ■  ■■    ■ ■■           ■■■
#                                                                                      ■       ■■■■■■■■■■    ■    ■    ■■    ■               ■ 
#                                                                                              ■             ■          ■■■■■■                 
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""";debug = True;""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

# todo 衝突判定+TOF制限を超えない高さ調節

# TTCの警告しきい値
TTC_THRESHOLD = 0.075
# 矢印判定のしきい値
ARROW_Z_THRESHOLD = 80000 # 約35cm以上
ARROW_X_THRESHOLD = 0 # 中央
# 矢印前後位置調整用誤差範囲
ARROW_Z_ERROR_RANGE = 20000 # 80000-100000
ARROW_X_ERROR_RANGE = 50 # -50-50
# 高さ調整のしきい値
TOF_THRESHOLD = 100
# 高さ調節の誤差範囲
TOF_ERROR_RANGE = 0 # 115-125

# 速度
X_SPEED = 5
Z_SPEED = 8
TOF_SPEED = 10

ready = False

frame_buffer = []

close_count = 0
is_avoid = False

arrow_count = 0
old_direction = ""
arrow_z = 0
arrow_x = 0
is_turn = False
turn_approved = False

is_moving = False
is_exit = False

frame_queue = queue.Queue()
g_frame = None

# TCPクライアントのセットアップ
command = command.Command()
command.connect('127.0.0.1', 8989, 1024, debug)

# TTC用パラメータの初期化
ttc = ttc.TTC()

# Status取得用
status = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
status.connect(('127.0.0.1', 8990))

# ToFセンサーの値
tof = TOF_THRESHOLD

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
    command.send(f"rc {erlon} {elevator} {srotol} {lador}", True)

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
    global is_moving, is_turn, old_direction, is_avoid, is_exit, turn_approved, arrow_x, arrow_z

    while True:
        if not ready:
            continue

        if is_exit:
            rc("0", "0", "0", "0")
            land()
            break

        if not is_moving:
            is_moving = True
            takeoff()
            time.sleep(2) # 2秒待機
            continue

        if is_moving & is_turn:
            if turn_approved:
                if(old_direction == "Left"):
                    rc("0", "0", "0", "0")
                    ccw("90")
                    time.sleep(2) # 2秒待機
                    is_turn = False
                    turn_approved = False
                    continue

                elif(old_direction == "Right"):
                    rc("0", "0", "0", "0")
                    cw("90")
                    time.sleep(2) # 2秒待機
                    is_turn = False
                    turn_approved = False
                    continue
            
            else:
                if(arrow_x < ARROW_X_THRESHOLD - ARROW_X_ERROR_RANGE):
                    print(f"左へ {arrow_x}")
                    if(tof < TOF_THRESHOLD - TOF_ERROR_RANGE):
                        rc(f"{-X_SPEED}", "0", f"{TOF_SPEED}", "0")
                    elif(tof > TOF_THRESHOLD + TOF_ERROR_RANGE):
                        rc(f"{-X_SPEED}", "0", f"{-TOF_SPEED}", "0")
                    else:
                        rc(f"{-X_SPEED}", "0", "0", "0")
                    
                elif(arrow_x > ARROW_X_THRESHOLD + ARROW_X_ERROR_RANGE):
                    print(f"右へ {arrow_x}")
                    if(tof < TOF_THRESHOLD - TOF_ERROR_RANGE):
                        rc(f"{X_SPEED}", "0", f"{TOF_SPEED}", "0")
                    elif(tof > TOF_THRESHOLD + TOF_ERROR_RANGE):
                        rc(f"{X_SPEED}", "0", f"{-TOF_SPEED}", "0")
                    else:
                        rc(f"{X_SPEED}", "0", "0", "0")

                elif(arrow_z < ARROW_Z_THRESHOLD - ARROW_Z_ERROR_RANGE):
                    print(f"前へ {arrow_z}")
                    if(tof < TOF_THRESHOLD - TOF_ERROR_RANGE):
                        rc("0", f"{Z_SPEED}", f"{TOF_SPEED}", "0")
                    elif(tof > TOF_THRESHOLD + TOF_ERROR_RANGE):
                        rc("0", f"{Z_SPEED}", f"{-TOF_SPEED}", "0")
                    else:
                        rc("0", f"{Z_SPEED}", "0", "0")

                elif(arrow_z > ARROW_Z_THRESHOLD + ARROW_Z_ERROR_RANGE):
                    print(f"後ろへ {arrow_z}")
                    if(tof < TOF_THRESHOLD - TOF_ERROR_RANGE):
                        rc("0", f"{-Z_SPEED}", f"{TOF_SPEED}", "0")
                    elif(tof > TOF_THRESHOLD + TOF_ERROR_RANGE):
                        rc("0", f"{-Z_SPEED}", f"{-TOF_SPEED}", "0")
                    else:
                        rc("0", f"{-Z_SPEED}", "0", "0")
                    
                else:
                    print(f"OK")
                    rc("0", "0", "0", "0")
                    turn_approved = True
            continue

        if is_moving & is_avoid:
            rc("0", "0", "0", "0")
            time.sleep(5) # 5秒待機
            is_avoid = False
            continue

        if is_moving:
            if(tof < TOF_THRESHOLD - TOF_ERROR_RANGE):
                rc("0", f"{Z_SPEED}", f"{TOF_SPEED}", "0")
            elif(tof > TOF_THRESHOLD + TOF_ERROR_RANGE):
                rc("0", f"{Z_SPEED}", f"{-TOF_SPEED}", "0")
            else:
                rc("0", f"{Z_SPEED}", "0", "0")
            time.sleep(0.5) # 0.5待機
            continue

        time.sleep(0.1)

dorone_thread = threading.Thread(target=move_drone)
dorone_thread.start()

def calc_ttc():
    """
    ttc計算処理
    """
    global g_frame, frame_buffer, close_count, is_avoid, is_turn, is_exit
    
    while not is_exit:
        if g_frame is None:
            continue

        # TTC計算
        ttc_value = ttc.analysis(g_frame)
        if ttc_value is not None:
            frame_buffer.append(ttc_value)

        # TTCの平均を計算して警告を出す
        if len(frame_buffer) == 5:
            average_ttc = np.mean(frame_buffer)
            #print(f"TTC: {average_ttc}")

            # 至近距離判定カウント
            if average_ttc < TTC_THRESHOLD:
                print(f"TTC: {average_ttc}")
                print("TTC: Object is very close.")
                close_count += 1
                if((not is_avoid) & (not is_turn)):
                    # 5連続以上で接近判定になったら停止させ回避行動を開始
                    if(close_count >= 5):
                        is_avoid = True
            else:
                close_count = 0

            frame_buffer = []

        time.sleep(0.1)

# ttc_thread = threading.Thread(target=calc_ttc)
# ttc_thread.start()

def calc_arrow():
    """
    矢印判定処理
    """
    global g_frame, arrow_count, old_direction, is_turn, is_avoid, is_exit, frame_queue, arrow_z, arrow_x

    while not is_exit:
        if g_frame is None:
            continue

        # 矢印判定とカメラ映像取得
        new_frame, direction, relative, position = arrow.Arrow().analysis(g_frame)
        if((direction is not None) & (relative is not None) & (position is not None)):
            lr, ud, dx, dy = position
            show_arrow_info(new_frame, f"Arrow:{direction}, X:{dx}({lr}), Y:{dy}({ud}), Z:{relative}")

            arrow_x = dx
            arrow_z = relative

            if(old_direction == direction): arrow_count += 1
            old_direction = direction

            # 5回連続で方向検知で方向転換
            if(not is_avoid):
                if(arrow_count >= 5):
                    is_turn = True
                else:
                    is_turn = False

        else:
            is_turn = False
            arrow_count = 0
            old_direction = ""

        frame_queue.put(new_frame)

        time.sleep(0.1)

arrow_thread = threading.Thread(target=calc_arrow)
arrow_thread.start()

def camera_thread():
    """
    カメラ映像を取得する
    """
    global g_frame

    # カメラストリームのセットアップ
    cap = cv2.VideoCapture('udp://@0.0.0.0:11113')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
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

def get_tof():
    """
    ToFセンサーの値を取得する
    """
    global is_exit, status, tof

    while not is_exit:
        try:
            data = status.recv(1024)
            data_str = data.decode("utf-8")
            decoder = json.JSONDecoder()
            parsed, _ = decoder.raw_decode(data_str)
            if isinstance(parsed, dict) and "time_of_flight" in parsed:
                tof = int(parsed["time_of_flight"])
            else:
                print("Received data is not in expected format:", parsed)


        except json.JSONDecodeError as e:
            print(f"JSON Decode Error: {e}")
        
        time.sleep(0.05) # 0.05秒ごとに処理

tof_thread = threading.Thread(target=get_tof)
tof_thread.start()

while True:
    frame = frame_queue.get()
    cv2.imshow("Tello-Detection", frame)
    if not ready: ready = True
    if cv2.waitKey(1) & 0xFF == 27:
        is_exit = True
        break

cv2.destroyAllWindows()
arrow_thread.join()
# ttc_thread.join()
dorone_thread.join()
video_thread.join()
tof_thread.join()
