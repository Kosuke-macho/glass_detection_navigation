#!/usr/bin/python3

import rospy
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Int16, Header

class SonarToPointCloud():
    def __init__(self):
        rospy.init_node('talk_sonar_pcl', anonymous=True)
        
        # PointCloud2のパブリッシャー
        self.pub_cloud = rospy.Publisher("sonar_pcl", PointCloud2, queue_size=1)
        
        # 3つの超音波センサーのサブスクライバー
        rospy.Subscriber("stm_pub_ultrasonic1", Int16, self.callback_front) # 正面
        rospy.Subscriber("stm_pub_ultrasonic2", Int16, self.callback_right) # 右斜め45度
        rospy.Subscriber("stm_pub_ultrasonic3", Int16, self.callback_left)  # 左斜め45度

        # 距離データ初期化 [cm] (-1は検知なし等の扱い)
        self.dist_front = 0
        self.dist_right = 0
        self.dist_left = 0
        
        # センサーの高さ設定 [m] (必要に応じて調整してください)
        self.sensor_z = 0.0 

        # 20Hz (0.05秒周期) で処理
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown():
            points = []
            
            # --- 1. 正面 (0度) の計算 ---
            if self.dist_front > 0: # 値が有効な場合
                d = self.dist_front / 100.0 # cm -> m 変換
                # 正面はX軸方向そのまま
                x = d
                y = 0.0
                points.append([x, y, self.sensor_z])

            # --- 2. 右斜め45度 (-45度) の計算 ---
            if self.dist_right > 0:
                d = self.dist_right / 100.0
                # ROS座標系: X前方, Y左 なので、右は角度マイナス
                angle = -math.radians(45) 
                x = d * math.cos(angle)
                y = d * math.sin(angle)
                points.append([x, y, self.sensor_z])

            # --- 3. 左斜め45度 (+45度) の計算 ---
            if self.dist_left > 0:
                d = self.dist_left / 100.0
                # ROS座標系: 左は角度プラス
                angle = math.radians(45)
                x = d * math.cos(angle)
                y = d * math.sin(angle)
                points.append([x, y, self.sensor_z])

            # --- PointCloudの作成とパブリッシュ ---
            # 点が1つ以上ある場合のみパブリッシュ
            if len(points) > 0:
                header = Header()
                header.frame_id = "sonar" # TFフレーム名 (rslidar等と合わせるなら適宜変更)
                header.stamp = rospy.Time.now() # 現在時刻

                # 点群メッセージを作成
                pcl_msg = pc2.create_cloud_xyz32(header, points)
                self.pub_cloud.publish(pcl_msg)
            
            rate.sleep()

    # --- コールバック関数 ---
    def callback_front(self, msg):
        self.dist_front = msg.data

    def callback_right(self, msg):
        self.dist_right = msg.data

    def callback_left(self, msg):
        self.dist_left = msg.data

if __name__ == '__main__':
    try:
        SonarToPointCloud()
    except rospy.ROSInterruptException:
        pass