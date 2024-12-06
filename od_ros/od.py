import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import math
import odrive
from odrive.enums import *

class OmniMotorController(Node):
    def __init__(self, config_file):
        super().__init__('omni_motor_controller')

        # 設定ファイルの読み込み
        self.get_logger().info(f"設定ファイルを読み込んでいます: {config_file}")
        with open(config_file, "r") as file:
            self.config = json.load(file)

        # モーター設定をクラス内で保持
        self.motor_configs = self.config.get("motors", [])
        if not self.motor_configs:
            self.get_logger().error("設定ファイルに 'motors' が存在しません。処理を中断します。")
            raise ValueError("設定ファイルが無効です。")

        # ODriveインスタンス
        self.odrives = {}
        self.connect_odrives()

        # /cmd_velトピックの購読
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )

    def connect_odrives(self):
        """
        設定ファイルに基づいてODriveコントローラーに接続します。
        """
        for motor_config in self.motor_configs:
            serial_number = motor_config["serial_number"]

            if serial_number not in self.odrives:
                self.get_logger().info(f"シリアル番号 {serial_number} のODriveに接続しています...")
                try:
                    self.odrives[serial_number] = odrive.find_any(serial_number=serial_number)
                    self.get_logger().info(f"ODrive {serial_number} に接続しました。")
                except Exception as e:
                    self.get_logger().error(f"ODrive {serial_number} に接続できません: {e}")
                    raise

    def set_motor_speed(self, serial_number, axis, speed):
        """
        指定されたモーターに速度を設定します。
        """
        odrv = self.odrives[serial_number]
        motor = odrv.axis0 if axis == 0 else odrv.axis1

        motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        motor.controller.input_vel = speed

        self.get_logger().info(f"シリアル {serial_number}, 軸 {axis} のモーター速度を {speed} に設定しました。")

    def cmd_vel_callback(self, msg):
        """
        /cmd_velメッセージを受け取り、オムニホイールロボットのモーター速度を計算して設定します。
        """
        x = msg.linear.x
        y = msg.linear.y

        # 各ホイールの速度を計算
        angles = [math.radians(45), math.radians(135), math.radians(225), math.radians(315)]
        speeds = [
            -math.sin(angle) * x + math.cos(angle) * y
            for angle in angles
        ]

        # 各モーターに速度を適用
        for i, motor_config in enumerate(self.motor_configs):
            serial_number = motor_config["serial_number"]
            axis = motor_config["axis"]
            self.set_motor_speed(serial_number, axis, speeds[i])

def main(args=None):
    rclpy.init(args=args)

    # 設定ファイルパラメータを解析するノードを初期化
    node = Node('argument_parser')

    # コマンドライン引数から設定ファイルを取得
    config_file = node.declare_parameter('config_file', '').value
    if not config_file:
        node.get_logger().error("パラメータ 'config_file' が指定されていません！ '--ros-args --param config_file:=<path_to_config>' を使用してください。")
        return

    try:
        motor_controller = OmniMotorController(config_file)
        rclpy.spin(motor_controller)
    except Exception as e:
        node.get_logger().error(f"ノードの起動中にエラーが発生しました: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
