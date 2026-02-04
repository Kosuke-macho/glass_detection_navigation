#!/usr/bin/python3
import rospy
import numpy as np
import tf
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid

class GlassMapGenerator:
    def __init__(self):
        rospy.init_node('glass_map_generator', anonymous=True)


        self.map_resolution = 0.05
        self.map_size = 50.0
        self.grid_size = int(self.map_size / self.map_resolution)
        self.origin_offset = self.map_size / 2.0
        
        # 初期値 -1 (未知)
        self.glass_map = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)
        
        self.tf_listener = tf.TransformListener()
        
        self.map_pub = rospy.Publisher("/glass_wall_map", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("/filtered_lidar_points", PointCloud2, self.cloud_callback)

        rospy.Timer(rospy.Duration(0.5), self.publish_map_timer)
        rospy.Timer(rospy.Duration(0.2), self.clear_footprint)

        self.need_publish = False 

        # --- ★追加: タイマー管理用変数 ---
        self.door_open_start_time = None  # 開放検知の開始時刻
        self.confirm_duration = rospy.Duration(2.0) # 確定にかかる時間 (2秒)

        rospy.loginfo(f"🟢 GlassMapGenerator: 慎重モード起動 (確定まで{self.confirm_duration.to_sec()}秒)")
        rospy.spin()

    def cloud_callback(self, msg):
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(gen))
        if len(points) == 0: return
        points = points[::5] 

        dists = np.linalg.norm(points[:, :2], axis=1)
        avg_dist = np.mean(dists)

        # --- ★修正: 判定ロジックに時間経過を追加 ---
        if avg_dist > 5.0:
            # 仮想点が出ている (ドアが開いている可能性がある)
            
            # まだ計測開始していなければ、今の時間を記録
            if self.door_open_start_time is None:
                self.door_open_start_time = rospy.Time.now()
                rospy.loginfo("⏳ ドア開放検知... 計測開始")

            # 経過時間を計算
            elapsed = rospy.Time.now() - self.door_open_start_time

            # 2秒以上経過したか？
            if elapsed > self.confirm_duration:
                rospy.loginfo_throttle(1.0, "モード: 🟢 クリアリング (確定！)")
                self.clear_door_rectangle(msg.header.frame_id)
            else:
                # まだ確定ではないので何もしない (待機)
                rospy.loginfo_throttle(0.5, f"⏳ 判定中... {elapsed.to_sec():.1f}s / 2.0s")

        else:
            # ガラス検知 (壁がある)
            # ★重要: 壁を見つけたらタイマーをリセットする！
            # これにより「2秒連続」という条件が守られる
            if self.door_open_start_time is not None:
                rospy.loginfo("❌ 開放キャンセル: 壁を検知しました")
                self.door_open_start_time = None # リセット

            rospy.loginfo_throttle(1.0, "モード: 🔴 壁プロット (Glass Detect)")
            self.mark_points_as_wall(points, msg.header.frame_id)

    # --- 以下、変更なし ---
    def clear_door_rectangle(self, frame_id):
        try:
            x_range = np.arange(0.5, 1.5, 0.05) 
            y_range = np.arange(-0.5, 0.5, 0.05)
            X, Y = np.meshgrid(x_range, y_range)
            local_points = np.column_stack((X.flatten(), Y.flatten(), np.zeros(X.size)))
            self.update_map_with_tf(local_points, frame_id, cost=0)
        except: pass

    def clear_footprint(self, event):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            cx, cy = trans[0], trans[1]
            r = 0.4
            step = 0.05
            x_range = np.arange(cx - r, cx + r, step)
            y_range = np.arange(cy - r, cy + r, step)
            X, Y = np.meshgrid(x_range, y_range)
            world_points = np.column_stack((X.flatten(), Y.flatten()))
            self.update_map_simple(world_points, cost=0)
        except: pass

    def mark_points_as_wall(self, points, frame_id):
        self.update_map_with_tf(points, frame_id, cost=100)

    def update_map_with_tf(self, local_points, frame_id, cost):
        try:
            if len(local_points) == 0: return
            (trans, rot) = self.tf_listener.lookupTransform('/map', frame_id, rospy.Time(0))
            matrix = self.tf_listener.fromTranslationRotation(trans, rot)
            points_np = np.array(local_points)
            if points_np.shape[1] == 2: points_np = np.hstack((points_np, np.zeros((points_np.shape[0], 1))))
            ones = np.ones((points_np.shape[0], 1))
            points_hom = np.hstack((points_np, ones))
            world_points = matrix.dot(points_hom.T).T
            self.apply_to_grid(world_points, cost)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

    def update_map_simple(self, world_points_list, cost):
        if isinstance(world_points_list, np.ndarray):
            world_points = world_points_list
        else:
            world_points = np.array(world_points_list)
        self.apply_to_grid(world_points, cost)

    def apply_to_grid(self, world_points, cost):
        map_x = ((world_points[:, 0] + self.origin_offset) / self.map_resolution).astype(int)
        map_y = ((world_points[:, 1] + self.origin_offset) / self.map_resolution).astype(int)
        valid_mask = (map_x >= 0) & (map_x < self.grid_size) & (map_y >= 0) & (map_y < self.grid_size)
        target_x = map_x[valid_mask]
        target_y = map_y[valid_mask]
        if len(target_x) == 0: return

        if cost == 100:
            current_vals = self.glass_map[target_y, target_x]
            write_mask = (current_vals != 0) 
            target_x = target_x[write_mask]
            target_y = target_y[write_mask]
            if len(target_x) == 0: return

        self.glass_map[target_y, target_x] = cost 
        self.need_publish = True

    def publish_map_timer(self, event):
        if self.need_publish:
            msg = OccupancyGrid()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.info.resolution = self.map_resolution
            msg.info.width = self.grid_size
            msg.info.height = self.grid_size
            msg.info.origin.position.x = -self.origin_offset
            msg.info.origin.position.y = -self.origin_offset
            msg.data = self.glass_map.flatten().tolist()
            self.map_pub.publish(msg)
            self.need_publish = False

if __name__ == '__main__':
    GlassMapGenerator()