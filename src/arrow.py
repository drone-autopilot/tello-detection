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
    
    def is_arrow_shape(self, approx):
        """
        90度程度が4つ、60度程度が3つでTRUE
        """
        if len(approx) != 7:
            return False
        
        acute_angles = 0  # 90度以下の角
        obtuse_angles = 0 # 90度以上の角

        for i in range(7):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % 7][0]
            p3 = approx[(i + 2) % 7][0]

            v1 = p2 - p1
            v2 = p3 - p2

            angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
            angle_degree = np.degrees(angle)

            inner_angle = 180 - angle_degree
            if (40 <= inner_angle) and (inner_angle <= 74) :
                acute_angles += 1
            elif (75 <= inner_angle) and (inner_angle <= 110):
                obtuse_angles += 1

        return acute_angles == 3 and obtuse_angles == 4
    
    def check_arrow_perspective(self, approx):
        """
        approx配列から矢印の傾き（perspective）をチェック
        """
        if approx is None or len(approx) < 4:
            return None

        # 四角形の辺の長さを計算
        # approx[2] から approx[5] までの頂点を使用
        edge_lengths = [np.linalg.norm(approx[i][0] - approx[(i+1)%4 + 2][0]) for i in range(2, 6)]

        # 辺の長さの比較
        length_ratio1 = edge_lengths[0] / edge_lengths[2]  # 上辺と下辺
        length_ratio2 = edge_lengths[1] / edge_lengths[3]  # 左辺と右辺

        return [length_ratio1, length_ratio2]
    
    def analysis(self, frame):
        """
        矢印検知
        """
        # バイラテラルフィルタを適用
        filtered_frame = cv2.bilateralFilter(frame, 9, 75, 75)

        # 二値化
        gray = cv2.cvtColor(filtered_frame, cv2.COLOR_BGR2GRAY)
        dx = cv2.Sobel(gray, cv2.CV_8U, 1, 0)
        dy = cv2.Sobel(gray, cv2.CV_8U, 0, 1)
        sobel = (np.sqrt(dx * dx + dy * dy) * 128.0).astype('uint8')
        _, sobel = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        # エッジ検出（閾値min, max）
        canny_img = cv2.Canny(sobel, 400, 800)

        # 輪郭を太くする
        kernel = np.ones((3,3), np.uint8)
        dilated_img = cv2.dilate(canny_img, kernel, iterations=1)

        # 輪郭検出
        contours, hierarchy = cv2.findContours(dilated_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # 最大面積の輪郭を探します
        contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)[:12]  # 最大面積の輪郭を12つまで取得

        max_contour = None
        max_approx = None  # 近似輪郭も保存

        arrow_i = -1
        contour_i = 0
        for contour in contours_sorted:
            epsilon = 0.03 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 7:  # 7角形を探す
                if self.is_arrow_shape(approx):
                    max_contour = contour
                    max_approx = approx
                    break
                else:
                    cv2.drawContours(frame, contour, -1, (0, 0, 255), 3)
                    arrow_i = contour_i
            #else:
                #cv2.drawContours(frame, contour, -1, (0, 255, 255), 3)
            contour_i = contour_i + 1 
                    
        contour_i = 0
        for contour2 in contours_sorted:
            if(not contour_i == arrow_i):
                cv2.drawContours(frame, contour2, -1, (0, 255, 255), 3)
                break
            contour_i = contour_i + 1 

        # 面積が最大の輪郭
        # max_contour = max(contours, key=lambda x: cv2.contourArea(x))

        # 点判定
        # epsilon = 0.03 * cv2.arcLength(max_contour, True)
        # approx = cv2.approxPolyDP(max_contour, epsilon, True)

        arrow_direction = None
        relative_size = None
        position_info = None
        perspective = None

        # n_gon = len(approx)
        # if n_gon == 7:
        if max_contour is not None:
            #for p in max_contour:
                #cv2.circle(frame, p[0], 4, (0, 255, 0), -1)
            cv2.drawContours(frame, max_contour, -1, (0, 255, 0), 3)

            # 矢印の歪み計算
            perspective = self.check_arrow_perspective(max_approx)

            if True is True:
                # モーメントを計算
                M = cv2.moments(max_approx)
                if M["m00"] != 0:
                    # 重心を計算（モーメントから中心座標を導出）
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                else:
                    # 重心を見つけることができない場合は、エラーを返すか、デフォルト値を設定
                    return frame, None, None, None, None
                
                # 画像のサイズを取得
                frame_height, frame_width = frame.shape[:2]

                # 矢印の先端を見つける
                arrow_direction = self.determine_arrow_direction(max_approx, center)

                # 矢印のバウンディングボックスを approx から取得
                x_min, y_min, bounding_box_width, bounding_box_height = self.get_bounding_box_from_approx(max_approx)
                # 矢印の相対的なサイズを取得（バウンディングボックスの面積）
                relative_size = bounding_box_width * bounding_box_height
                # 矢印の位置情報取得
                x_position, y_position, delta_x, delta_y = self.determine_arrow_position(center, frame_width, frame_height)
                position_info = (x_position, y_position, delta_x, delta_y)

        return frame, arrow_direction, relative_size, position_info, perspective