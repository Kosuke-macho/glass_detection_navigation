#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16
import dynamic_reconfigure.client
from collections import deque  # 履歴管理用

class SpeedController:
    def __init__(self):
        rospy.init_node('adaptive_speed_controller', anonymous=True)

        self.dwa_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS", timeout=30)

        self.sensor_values = {
            "sensor1": 9.99,
            "sensor2": 9.99,
            "sensor3": 9.99
        }

        # ★追加: ノイズ対策の設定
        # 何フレーム連続で検知したら反応するか
        # Arduinoが20Hzの場合、5フレーム = 約0.25秒
        self.history_length = 5 
        self.dist_history = deque(maxlen=self.history_length)

        # 初期値で履歴を埋めておく(安全側=9.99m)
        for _ in range(self.history_length):
            self.dist_history.append(9.99)

        rospy.Subscriber("stm_pub_ultrasonic1", Int16, self.callback_sensor1)
        rospy.Subscriber("stm_pub_ultrasonic2", Int16, self.callback_sensor2)
        rospy.Subscriber("stm_pub_ultrasonic3", Int16, self.callback_sensor3)

        self.current_mode = "unknown"

        rospy.loginfo(f"🚀 速度制御ノード起動: ノイズフィルタ付き (履歴:{self.history_length}フレーム)")
        rospy.spin()

    def callback_sensor1(self, msg):
        self.update_sensor_value("sensor1", msg.data)

    def callback_sensor2(self, msg):
        self.update_sensor_value("sensor2", msg.data)

    def callback_sensor3(self, msg):
        self.update_sensor_value("sensor3", msg.data)

    def update_sensor_value(self, sensor_name, raw_data):
        dist_m = raw_data / 100.0
        self.sensor_values[sensor_name] = dist_m
        self.process_speed_control()

    def process_speed_control(self):
        # 1. まず、今の瞬間の3つのセンサーの最小値(一番危ない値)を取る
        current_min_dist = min(self.sensor_values.values())

        # 2. 履歴に追加 (古いものは自動で押し出される)
        self.dist_history.append(current_min_dist)

        # 3. ★ノイズフィルタリング (ここが重要！)
        # 履歴の中で「最大の値(=一番安全な値)」を採用する。
        # つまり、履歴の中に1つでも「安全(遠い)」データがあれば、そちらを信じる。
        # 全てのデータが「危険(近い)」になって初めて、filtered_dist が小さくなる。
        filtered_dist = max(self.dist_history)
        
        target_vel = 0.5 
        mode = "SAFE"

        # 判定にはフィルタ済みの値を使う
        if filtered_dist < 0.4:
            mode = "STOP_WAIT"
            target_vel = 0.0
        elif filtered_dist < 0.8:
            mode = "CRAWL"
            target_vel = 0.1
        elif filtered_dist < 1.5:
            mode = "SLOW"
            target_vel = 0.25
        else:
            mode = "NORMAL"
            target_vel = 0.5

        if mode != self.current_mode:
            # デバッグ用に、生の値とフィルタ後の値を表示しておくと調整しやすいです
            rospy.loginfo(f"判定: Raw={current_min_dist:.2f}m -> Filtered={filtered_dist:.2f}m")
            self.change_speed(target_vel, mode)

    def change_speed(self, max_vel, mode_name):
        rospy.loginfo(f"🛑 速度変更: {mode_name} -> {max_vel} m/s")
        params = {
            'max_vel_x': max_vel,
        }
        self.dwa_client.update_configuration(params)
        self.current_mode = mode_name

if __name__ == '__main__':
    SpeedController()