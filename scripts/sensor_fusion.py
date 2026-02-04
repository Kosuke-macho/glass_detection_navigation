#!/usr/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import cKDTree
from std_msgs.msg import Header
import math
import numpy as np

class LiDARNearSonarFilter:
    def __init__(self):
        rospy.init_node('lidar_near_sonar_filter', anonymous=True)

        self.matching_threshold = 0.5
        self.sonar_points = []
        self.sonar_tree = None 
        
        self.filtered_pub = rospy.Publisher("/filtered_lidar_points", PointCloud2, queue_size=10)

        rospy.Subscriber("sonar_pcl", PointCloud2, self.sonar_callback)
        rospy.Subscriber("rslidar_points", PointCloud2, self.lidar_callback)

        rospy.loginfo("🟢 LiDARNearSonarFilter ノード起動中 (高速・シンプル版)")
        rospy.spin()

    def sonar_callback(self, msg):
        # 高速化のため、ソナー更新時のみTreeを再構築
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if points:
            self.sonar_points = points
            self.sonar_tree = cKDTree(self.sonar_points)

    def lidar_callback(self, msg):
        # ソナーデータがない場合は何もしない
        if self.sonar_tree is None:
            return

        # Numpyを使って高速に読み込み
        lidar_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not lidar_points:
            return
        lidar_np = np.array(lidar_points)

        # 高速フィルタリング (Numpy)
        dists, _ = self.sonar_tree.query(lidar_np, k=1, distance_upper_bound=self.matching_threshold)
        mask = dists < float('inf')
        filtered_points = lidar_np[mask]

        # --- シンプル化した仮想点群生成 ---
        # マッチする点がなければ、無条件で7m先に点を置く
        if len(filtered_points) == 0:
            # rospy.logwarn("⚠️ 近傍点なし: 前方7mに仮想点群を生成します")
            filtered_points = self.generate_virtual_points(
                radius=7.0,       # 前方距離
                angle_range=math.radians(60),
                num_points=15,
                z=0.0
            )
        # -------------------------------

        # パブリッシュ
        if len(filtered_points) > 0:
            header = Header()
            header.stamp = msg.header.stamp    # タイムスタンプ同期
            header.frame_id = msg.header.frame_id
            
            cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
            self.filtered_pub.publish(cloud_msg)

    def generate_virtual_points(self, radius, angle_range, num_points, z):
        """ シンプルな扇状点群生成 """
        points = []
        start_angle = -angle_range / 2
        step = angle_range / (num_points - 1)
        for i in range(num_points):
            angle = start_angle + i * step
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            points.append([x, y, z])
        return points

def main():
    LiDARNearSonarFilter()

if __name__ == '__main__':
    main()