import cv2
import numpy as np

class Arrow:
    def determine_arrow_direction(self, approx, center):
        """
        approx配列から矢印の方向を判定
        """
        if center is None: return None
    
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
    
    def get_bounding_box_from_approx(self, approx):
        """
        approx配列からバウンディングボックスを計算
        """
        # xとyの座標をそれぞれ抽出
        x_coords = [point[0][0] for point in approx]
        y_coords = [point[0][1] for point in approx]

        # 最小値と最大値を取得
        x_min, x_max = min(x_coords), max(x_coords)
        y_min, y_max = min(y_coords), max(y_coords)

        # バウンディングボックスの幅と高さを計算
        bounding_box_width = x_max - x_min
        bounding_box_height = y_max - y_min

        return x_min, y_min, bounding_box_width, bounding_box_height
    
    def determine_arrow_position(self, center, frame_width, frame_height):
        """
        矢印の中心点から矢印の位置を計算
        """
        if center is None: return None

        # 画像の中心座標を取得
        image_center_x = frame_width // 2
        image_center_y = frame_height // 2
        
        # 矢印の重心と画像の中心との差を計算
        delta_x = center[0] - image_center_x
        delta_y = center[1] - image_center_y
        
        # X方向（左右）の位置
        if delta_x < 0:
            x_position = 'Left'
        else:
            x_position = 'Right'
        
        # Y方向（上下）の位置
        if delta_y < 0:
            y_position = 'Up'
        else:
            y_position = 'Down'
        
        return x_position, y_position, delta_x, delta_y
    
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

        arrow_direction = None
        relative_size = None
        position_info = None

        # 点判定
        arclen = cv2.arcLength(max_contour, True)
        #approx = cv2.approxPolyDP(max_contour, arclen * 1.0e-2, True)
        epsilon = 0.03 * cv2.arcLength(max_contour, True)
        approx = cv2.approxPolyDP(max_contour, epsilon, True)
        n_gon = len(approx)
        if n_gon == 7:
            for p in max_contour:
                cv2.circle(frame, p[0], 4, (0, 255, 0), -1)

            # モーメントを計算
            M = cv2.moments(approx)
            if M["m00"] != 0:
                # 重心を計算（モーメントから中心座標を導出）
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                # 重心を見つけることができない場合は、エラーを返すか、デフォルト値を設定
                return None
            
            # 画像のサイズを取得
            frame_height, frame_width = frame.shape[:2]

            # 矢印の先端を見つける
            arrow_direction = self.determine_arrow_direction(approx, center)

            # 矢印のバウンディングボックスを approx から取得
            x_min, y_min, bounding_box_width, bounding_box_height = self.get_bounding_box_from_approx(approx)
            # 矢印の相対的なサイズを取得（バウンディングボックスの面積）
            relative_size = bounding_box_width * bounding_box_height
            # 矢印の位置情報取得
            x_position, y_position, delta_x, delta_y = self.determine_arrow_position(center, frame_width, frame_height)
            position_info = (x_position, y_position, delta_x, delta_y)

        return frame, arrow_direction, relative_size, position_info