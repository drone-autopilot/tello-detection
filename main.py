import cv2
import numpy as np
import time

# キャプチャ用のオブジェクト
cap = cv2.VideoCapture('udp://@0.0.0.0:11113')
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 270)

# カメラの内部パラメータ（Telloカメラの720p設定）
fx = 916.0  # 焦点距離 (ピクセル)
fy = 914.0  # 焦点距離 (ピクセル)
cx = 319.5 / 2  # 主点のX座標 (ピクセル)
cy = 239.5 / 2  # 主点のY座標 (ピクセル)

# カメラ行列
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]], dtype=np.float32)

# オプティカルフローの初期化
prev_frame = None
lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# ターゲットオブジェクトの実際の幅（メートル）
object_width = 0.1  # 仮のオブジェクト幅を設定します

# 閾値
TTC_THRESHOLD = 0.13  # TTCが0.12未満の場合にアラート表示

frame_count = 0
prev_time = time.time()
frame_buffer = []

# モルフォロジー変換のカーネル
kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()
    
    # 動画フレームが空ならスキップ
    if frame is None or frame.size == 0:
        continue
    
    # 時間計測
    current_time = time.time()
    elapsed_time = current_time - prev_time
    
    if elapsed_time >= 0.1:
        
        # ガウシアンフィルタを適用
        blurred_frame = cv2.GaussianBlur(frame, (7, 7), 0)

        # グレースケールに変換
        gray = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)

        # エッジ検出
        edges = cv2.Canny(gray, 30, 70)

        if prev_frame is not None:
            
            # バイラテラルフィルタでノイズ減少
            frame_diff = cv2.bilateralFilter(edges, 9, 75, 75)
            _, binary_diff = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)

            cv2.imshow("TTC Calculation", binary_diff)

            prevPts = cv2.goodFeaturesToTrack(binary_diff, mask=None, maxCorners=150, qualityLevel=0.4, minDistance=5, blockSize=7)

            # prevPtsがNoneでないことを確認
            if prevPts is not None:
                prevPts = prevPts.astype(np.float32)
                p1, st, err = cv2.calcOpticalFlowPyrLK(prev_frame, binary_diff, prevPts, None, **lk_params)

                # 有効なフローを選択
                valid_flow = p1[st == 1]
    
                if valid_flow.size > 0:
                    # オブジェクトの速度ベクトルを計算
                    object_velocity = np.mean(valid_flow, axis=0)
                    object_speed = np.linalg.norm(object_velocity)
        
                    # TTCを計算
                    ttc = (object_width * fx) / object_speed if object_speed > 0 else float('inf')
                    frame_buffer.append(ttc)

    
        prev_frame = edges
        prev_time = current_time
        
    frame_count += 1
    
    if len(frame_buffer) == 5:
        # 直近5フレームのTTCの平均を計算
        average_ttc = np.mean(frame_buffer)
        print(average_ttc)
        if average_ttc < TTC_THRESHOLD:
            print("oh! Object is very close.")
        frame_buffer = []
    
    # 画面表示
    #cv2.imshow("TTC Calculation", frame)
    
    # キー入力で終了
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cap.release()
cv2.destroyAllWindows()
