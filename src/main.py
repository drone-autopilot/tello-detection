from PIL import Image, ImageDraw, ImageFont
import cv2
import numpy as np
import time
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

# 矢印判定のしきい値
ARROW_Z_THRESHOLD = 80000
ARROW_X_THRESHOLD = 0 # 中央
ARROW_Y_THRESHOLD = 0
# 矢印前後位置調整用誤差範囲
ARROW_Z_ERROR_RANGE = 20000 # 80000-100000
ARROW_X_ERROR_RANGE = 50 # -50-50
ARROW_Y_ERROR_RANGE = 130
# 高さ調整のしきい値
TOF_THRESHOLD = 100
# 高さ調節の誤差範囲
TOF_ERROR_RANGE = 5 # 95-105

# 速度
X_LOW_SPEED = 4
X_HIGH_SPEED = 8
Z_LOW_SPEED = 5
Z_HIGH_SPEED = 15
Z_BACK_SPEED = 5
Y_LOW_SPEED = 10
Y_HIGH_SPEED = 13
TOF_SPEED = 13
TURN_SPEED = 10

MANUAL_SPEED = 15
MANUAL_TURN = 35

ready = False

frame_buffer = []

arrow_count = 0
old_direction = ""
arrow_z = 0
arrow_x = 0
arrow_y = 0
arrow_pers = [0,0]
is_turn = False
turn_approved = False

is_moving = False
is_exit = False
is_manual = False

frame_queue = queue.Queue()
g_frame = None

drone_info = "待機中"

# TCPクライアントのセットアップ
command = command.Command()
command.connect('127.0.0.1', 8989, 1024, debug)

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

def show_drone_info(frame):
    """
    cv2の画面内にドローンの状態や命令を表示します
    """
    global drone_info

    # PILで日本語対応フォントを設定
    font_path = "C:\Windows\Fonts\meiryo.ttc"  # 日本語フォントのパスを設定
    font_size = 30
    font = ImageFont.truetype(font_path, font_size)

    # OpenCV画像をPIL画像に変換
    pil_img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)

    # テキストの位置を設定
    text_offset_x = 10
    text_offset_y = 670
    if(is_manual):
        text_offset_y = 630

    # 縁取り文字の設定
    text_color = (255, 255, 255)  # 白色
    outline_color = (0, 0, 0)  # 黒色
    thickness = 2

    # 縁取り文字を描画
    for x in range(-thickness, thickness+1):
        for y in range(-thickness, thickness+1):
            draw.text((text_offset_x+x, text_offset_y+y), drone_info, font=font, fill=outline_color)

    # 通常のテキストを描画
    draw.text((text_offset_x, text_offset_y), drone_info, font=font, fill=text_color)

    # PIL画像をOpenCV画像に変換
    frame = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

    return frame

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

def rc(erlon: str, elevator: str, srotol: str, lador: str, skip: bool = True):
    """
    rcコマンドを送信
    erlon: 正で右移動、負で左移動
    elevator: 正で前進、負で後進
    srotol: 正で上昇、負で下降
    lador: 正で右旋回、負で左旋回
    """
    if(skip == False):
        while(True):
            result = command.send(f"rc {erlon} {elevator} {srotol} {lador}")
            if(result): break
            time.sleep(0.1)
    else:
        command.send(f"rc {erlon} {elevator} {srotol} {lador}", True)

def cw(degree: str):
    """
    cwコマンドを送信
    時計回りに指定した角度分旋回する
    """
    while(True):
        rc("0", "0", "0", "0")
        time.sleep(0.5)
        result = command.send(f"cw {degree}")
        if(result): break
        time.sleep(0.1)

def ccw(degree: str):
    """
    ccwコマンドを送信
    反時計回りに指定した角度分旋回する
    """
    while(True):
        rc("0", "0", "0", "0")
        time.sleep(0.5)
        result = command.send(f"ccw {degree}")
        if(result): break
        time.sleep(0.1)

def move_drone():
    """
    ドローンの制御関数
    """
    global is_moving, is_turn, old_direction, is_exit, turn_approved, arrow_x, arrow_z, arrow_y, drone_info

    while True:
        if is_manual:
            drone_info = "マニュアルモード [V:停止] [WASD:前左後右] [QE:左旋右旋]\n　　　　　　　　 [Space:上] [TAB:下] [R:着陸] [F:離陸] [M:終了]"
            time.sleep(0.1)
            continue

        if not ready:
            time.sleep(0.1)
            continue

        if is_exit:
            drone_info = "停止"
            rc("0", "0", "0", "0")
            land()
            break

        if not is_moving:
            drone_info = "離陸する"
            is_moving = True
            takeoff()
            time.sleep(5) # 2秒待機
            continue

        if is_moving & is_turn:
            if turn_approved:
                if(old_direction == "Left"):
                    drone_info = "左へ回転"
                    ccw("90")
                    time.sleep(2) # 2秒待機
                    is_turn = False
                    turn_approved = False
                    continue

                elif(old_direction == "Right"):
                    drone_info = "右へ回転"
                    cw("90")
                    time.sleep(2) # 2秒待機
                    is_turn = False
                    turn_approved = False
                    continue
            
            else:
                deltas = {
                    "y_diff": abs(arrow_y) - ARROW_Y_THRESHOLD,
                    "x_diff": abs(arrow_x) - ARROW_X_THRESHOLD
                }
                # 一番ズレの大きい方向を優先して移動する(距離は除外)
                if(arrow_y < ARROW_Y_THRESHOLD - ARROW_Y_ERROR_RANGE):
                    if(max(deltas, key=deltas.get) == "y_diff"):
                        drone_info = "上昇する"
                        print(f"上へ {arrow_y}")
                        if(arrow_z < 30000):
                            rc("0", "0", f"{Y_HIGH_SPEED}", "0")
                        else:
                            rc("0", "0", f"{Y_LOW_SPEED}", "0")

                elif(arrow_y > ARROW_Y_THRESHOLD + ARROW_Y_ERROR_RANGE):
                    if(max(deltas, key=deltas.get) == "y_diff"):
                        drone_info = "下降する"
                        print(f"下へ {arrow_y}")
                        if(arrow_z < 30000):
                            rc("0", "0", f"{-Y_HIGH_SPEED}", "0")
                        else:
                            rc("0", "0", f"{-Y_LOW_SPEED}", "0")

                if(arrow_x < ARROW_X_THRESHOLD - ARROW_X_ERROR_RANGE):
                    if(max(deltas, key=deltas.get) == "x_diff"):
                        drone_info = "左へ移動"
                        print(f"左へ {arrow_x}")
                        if(arrow_z < 30000):
                            rc(f"{-X_HIGH_SPEED}", "0", "0", "0")
                        else:
                            rc(f"{-X_LOW_SPEED}", "0", "0", "0")
                    
                elif(arrow_x > ARROW_X_THRESHOLD + ARROW_X_ERROR_RANGE):
                    if(max(deltas, key=deltas.get) == "x_diff"):
                        drone_info = "右へ移動"
                        print(f"右へ {arrow_x}")
                        if(arrow_z < 30000):
                            rc(f"{X_HIGH_SPEED}", "0", "0", "0")
                        else:
                            rc(f"{X_LOW_SPEED}", "0", "0", "0")

                elif(arrow_z < ARROW_Z_THRESHOLD - ARROW_Z_ERROR_RANGE):
                    drone_info = "前進する"
                    print(f"前へ {arrow_z}")
                    if(arrow_z < 30000):
                        rc("0", f"{Z_HIGH_SPEED}", "0", "0")
                    else:
                        rc("0", f"{Z_LOW_SPEED}", "0", "0")

                elif(arrow_z > ARROW_Z_THRESHOLD + ARROW_Z_ERROR_RANGE):
                    drone_info = "後退する"
                    print(f"後ろへ {arrow_z}")
                    rc("0", f"{-Z_BACK_SPEED}", "0", "0")
                    
                else:
                    # 矢印の歪みを確認　気休め程度の角度修正
                    if(old_direction == "Left"):
                        # 左向き矢印：(1.0~1.1 && 0.8~2.0)以内に 0.64未満3.0以上は反時計回り
                        if(arrow_pers[1] >= 0.8 and arrow_pers[1] < 2.0):
                            if(arrow_pers[0] < 1.0):
                                drone_info = "右旋回"
                                rc("0", "0", "0", f"{TURN_SPEED}")
                                time.sleep(0.5)
                                continue
                            elif(arrow_pers[0] >= 1.1):
                                drone_info = "左旋回"
                                rc("0", "0", "0", f"{-TURN_SPEED}")
                                time.sleep(0.5)
                                continue
                        elif(arrow_pers[1] >= 3.0):
                            if(arrow_pers[0] < 0.64):
                                drone_info = "左旋回"
                                rc("0", "0", "0", f"{-TURN_SPEED}")
                                time.sleep(0.5)
                                continue

                    elif(old_direction == "Right"):
                        # 右向き矢印：(1.0~1.1 && 0.8~2.0)以内に 0.63以上0.4未満は時計回り
                        if(arrow_pers[1] >= 0.8 and arrow_pers[1] < 2.0):
                            if(arrow_pers[0] < 1.0):
                                drone_info = "左旋回"
                                rc("0", "0", "0", f"{-TURN_SPEED}")
                                time.sleep(0.5)
                                continue
                            elif(arrow_pers[0] >= 1.1):
                                drone_info = "右旋回"
                                rc("0", "0", "0", f"{TURN_SPEED}")
                                time.sleep(0.5)
                                continue
                        elif(arrow_pers[1] < 0.4):
                            if(arrow_pers[0] >= 0.63):
                                drone_info = "右旋回"
                                rc("0", "0", "0", f"{TURN_SPEED}")
                                time.sleep(0.5)
                                continue

                    #arrow_pers[1] (0.95)1.0~1.1(1.15)の範囲内＋追加条件なら回転実行
                    if(not(arrow_pers[1] >= 0.95 and arrow_pers[1] < 1.15)):
                        if(old_direction == "Left"):
                            if(not(arrow_pers[1] > 3.0)):
                                time.sleep(0.5)
                                continue
                        elif(old_direction == "Right"):
                            if(not(arrow_pers[1] < 0.4)):
                                time.sleep(0.5)
                                continue

                    drone_info = "回転位置に到着"
                    print(f"OK")
                    rc("0", "0", "0", "0")
                    turn_approved = True

                time.sleep(0.5)
            continue

        if is_moving:
            if(tof < TOF_THRESHOLD - TOF_ERROR_RANGE):
                drone_info = "上昇する"
                rc("0", "0", f"{TOF_SPEED}", "0")
            elif(tof > TOF_THRESHOLD + TOF_ERROR_RANGE):
                drone_info = "下降する"
                rc("0", "0", f"{-TOF_SPEED}", "0")
            else:
                drone_info = "前進する"
                rc("0", f"{Z_HIGH_SPEED}", "0", "0")
            time.sleep(0.5) # 0.5待機
            continue

        time.sleep(0.1)

dorone_thread = threading.Thread(target=move_drone)
dorone_thread.start()

def calc_arrow():
    """
    矢印判定処理
    """
    global g_frame, arrow_count, old_direction, is_turn, is_exit, frame_queue, arrow_z, arrow_x, arrow_y, arrow_pers

    while not is_exit:
        if g_frame is None:
            continue

        # 矢印判定とカメラ映像取得
        new_frame, direction, relative, position, perspective = arrow.Arrow().analysis(g_frame)
        if((direction is not None) & (relative is not None) & (position is not None)):
            lr, ud, dx, dy = position
            show_arrow_info(new_frame, f"Arrow:{direction}, X:{dx}({lr}), Y:{dy}({ud}), Z:{relative}")

            arrow_x = dx
            arrow_y = dy
            arrow_z = relative
            arrow_pers = perspective

            if(old_direction == direction): arrow_count += 1
            old_direction = direction

            # 5回連続で方向検知で方向転換
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
    frame = show_drone_info(frame_queue.get())
    cv2.imshow("Tello-Detection", frame)
    if not ready: ready = True
    key = cv2.waitKey(1) & 0xFF

    if key == 27: #ESC
        is_manual = False
        is_exit = True
        break
    elif key == ord('m'):
        is_manual = not is_manual
        rc("0", "0", "0", "0", True)

    #以下、マニュアルモード時の動作
    elif (key == ord('w')) and is_manual:
        rc("0", f"{MANUAL_SPEED}", "0", "0")
        start_time = time.time() * 1000
    elif (key == ord('a')) and is_manual:
        rc(f"{-MANUAL_SPEED}", "0", "0", "0")
        start_time = time.time() * 1000
    elif (key == ord('s')) and is_manual:
        rc("0", f"{-MANUAL_SPEED}", "0", "0")
        start_time = time.time() * 1000
    elif (key == ord('d')) and is_manual:
        rc(f"{MANUAL_SPEED}", "0", "0", "0")
        start_time = time.time() * 1000
    elif (key == ord('q')) and is_manual:
        rc("0", "0", "0", f"{-MANUAL_TURN}")
        start_time = time.time() * 1000
    elif (key == ord('e')) and is_manual:
        rc("0", "0", "0", f"{MANUAL_TURN}")
        start_time = time.time() * 1000
    elif (key == ord(' ')) and is_manual:
        rc("0", "0", f"{MANUAL_SPEED}", "0")
        start_time = time.time() * 1000
    elif (key == 9) and is_manual: #TAB
        rc("0", "0", f"{-MANUAL_SPEED}", "0")
        start_time = time.time() * 1000
    elif (key == ord('f')) and is_manual:
        takeoff()
        start_time = time.time() * 1000
    elif (key == ord('r')) and is_manual:
        land()
        start_time = time.time() * 1000
    elif (key == ord('v')) and is_manual:
        rc("0", "0", "0", "0", True)
        start_time = time.time() * 1000

cv2.destroyAllWindows()
arrow_thread.join()
dorone_thread.join()
video_thread.join()
tof_thread.join()
