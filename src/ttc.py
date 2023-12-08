import cv2
import numpy as np

class TTC:
    def __init__(self):

        # カメラの内部パラメータ（Telloカメラの720p設定を480x270に調整）
        self.fx = 916.0
        self.fy = 916.0
        self.cx = 640.0  # 1280の半分
        self.cy = 360.0  # 720の半分

        # カメラ行列
        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                [0, self.fy, self.cy],
                                [0, 0, 1]], dtype=np.float32)

        # オプティカルフローの初期化
        self.prev_frame = None
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # ターゲットオブジェクトの実際の幅（メートル）
        self.object_width = 0.1  

        
        self.MAX_TTC = 5  # TTCがこの値より大きい場合は無視

    def analysis(self, frame):
        ttc = None

        # ガウシアンフィルタで画像を平滑化
        blurred_frame = cv2.GaussianBlur(frame, (7, 7), 0)
        
        # グレースケールに変換
        gray = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2GRAY)

        # エッジ検出
        low_threshold = 30
        high_threshold = 70
        edges = cv2.Canny(gray, low_threshold, high_threshold)

        # 前のフレームが存在する場合、オプティカルフローを計算
        if self.prev_frame is not None:
            frame_diff = cv2.bilateralFilter(edges, 9, 75, 75)
            _, binary_diff = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)

            # 特徴点の検出
            prevPts = cv2.goodFeaturesToTrack(binary_diff, mask=None, maxCorners=150, qualityLevel=0.4, minDistance=5, blockSize=7)

            # 特徴点が検出された場合、オプティカルフローを計算
            if prevPts is not None:
                prevPts = prevPts.astype(np.float32)
                p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_frame, binary_diff, prevPts, None, **self.lk_params)

                # 有効なフローを選択
                valid_flow = p1[st == 1]

                # 速度ベクトルを計算してTTCを算出
                if valid_flow.size > 0:
                    object_velocity = np.mean(valid_flow, axis=0)
                    object_speed = np.linalg.norm(object_velocity)

                    ttc = (self.object_width * self.fx) / object_speed if object_speed > 0 else self.MAX_TTC

        # 現在のフレームを保存
        self.prev_frame = edges

        return ttc