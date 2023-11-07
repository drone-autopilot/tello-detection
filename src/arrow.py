import cv2
import numpy as np

class Arrow:
    def determine_arrow_direction(self, approx):
        # モーメントを計算
        M = cv2.moments(approx)
        if M["m00"] != 0:
            # 重心を計算（モーメントから中心座標を導出）
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            # 重心を見つけることができない場合は、エラーを返すか、デフォルト値を設定
            return "error"
    
        # 輪郭点の凸包を計算
        hull = cv2.convexHull(approx, returnPoints=True)
        
        # 凸包の点のうち、最も先端と思われる点を見つける
        tip = None
        max_distance = 0
        for point in hull:
            point = point[0]
            distance = np.sqrt((point[0] - center[0])**2 + (point[1] - center[1])**2)
            if distance > max_distance:
                max_distance = distance
                tip = point
        
        # 矢印の先端から軸へのベクトルを計算
        vector = np.array(tip) - np.array(center)

        # ベクトルの方向に応じて矢印の向きを決定
        angle = np.arctan2(vector[1], vector[0])
        direction = None
        if -np.pi/4 <= angle < np.pi/4:
            direction = "Left"
        elif np.pi/4 <= angle < 3*np.pi/4:
            direction = "Up"
        elif -3*np.pi/4 <= angle < -np.pi/4:
            direction = "Down"
        else:
            direction = "Right"
        
        return direction
    
    def analysis(self, frame):
        # 中央値ブラー
        mask_img = cv2.medianBlur(frame, 5)

        # 二値化
        gray = cv2.cvtColor(mask_img, cv2.COLOR_BGR2GRAY)
        dx = cv2.Sobel(gray, cv2.CV_8U, 1, 0)
        dy = cv2.Sobel(gray, cv2.CV_8U, 0, 1)
        sobel = (np.sqrt(dx * dx + dy * dy) * 128.0).astype('uint8')
        _, sobel = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # エッジ検出（閾値min, max）
        canny_img = cv2.Canny(sobel, 400, 800)

        # エッジ強調
        height, width = canny_img.shape
        ksize = int(max(height, width) * 0.01)
        ksize = ksize // 2 * 2 + 1
        frame = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, np.ones((ksize, ksize), dtype='uint8'))

        # 輪郭検出
        contours, hierarchy = cv2.findContours(canny_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 面積が最大の輪郭
        max_contour = max(contours, key=lambda x: cv2.contourArea(x))

        arrow_direction = "none"

        # 点判定
        arclen = cv2.arcLength(max_contour, True)
        #approx = cv2.approxPolyDP(max_contour, arclen * 1.0e-2, True)
        epsilon = 0.03 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
        n_gon = len(approx)
        if n_gon == 7:
            for p in max_contour:
                cv2.circle(frame, p[0], 4, (0, 255, 0), -1)

            # 矢印の先端を見つける
            arrow_direction = self.determine_arrow_direction(approx)
            print(f"Arrow is pointing: {arrow_direction}")

        # 出力
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.imshow("img", frame)

        return arrow_direction