import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import json
import odrive
import math
from odrive.enums import *
from pynput import keyboard  # キーボード入力を監視

class OmniMotorController(Node):
    def __init__(self, config_file):
        super().__init__('omni_motor_controller')

        self.get_logger().info(f"設定ファイルを読み込んでいます: {config_file}")
        with open(config_file, "r") as file:
            self.config = json.load(file)

        self.motor_configs = self.config.get("motors", [])
        if not self.motor_configs:
            self.get_logger().error("設定ファイルに 'motors' が存在しません。処理を中断します。")
            raise ValueError("設定ファイルが無効です。")

        self.odrives = {}
        self.connect_odrives()

        self.subscription = self.create_subscription(
            Twist,
            '/pid_cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()
        self.calibration_done = False

        self.R = self.config.get("R", 0.2)  # 中心からホイールまでの距離(m)
        self.r = self.config.get("wheel_radius", 0.05)  # ホイール半径(m)

    def connect_odrives(self):
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

    def cmd_vel_callback(self, msg):
        """
        /pid_cmd_vel (Twist) メッセージを受け取ってモーター速度を計算し適用
        """
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega = msg.angular.z

        coeff = 1 / (2*math.pi*self.r)
        r2 = math.sqrt(2)
        v1 = coeff * ((-Vx + Vy)/r2 + self.R * omega)
        v2 = coeff * ((-Vx - Vy)/r2 + self.R * omega)
        v3 = coeff * ((Vx - Vy)/r2 + self.R * omega)
        v4 = coeff * ((Vx + Vy)/r2 + self.R * omega)

        speeds = [v1, v2, v3, v4]

        for i, motor_config in enumerate(self.motor_configs):
            serial_number = motor_config["serial_number"]
            axis = motor_config["axis"]
            self.set_motor_speed(serial_number, axis, speeds[i])

    def set_motor_speed(self, serial_number, axis, speed):
        if serial_number not in self.odrives:
            self.get_logger().error(f"シリアル {serial_number} のODriveが見つかりません。")
            return

        odrv = self.odrives[serial_number]
        motor = odrv.axis0 if axis == 0 else odrv.axis1

        motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        motor.controller.input_vel = speed

        self.get_logger().info(f"シリアル {serial_number}, 軸 {axis} のモーター速度を {speed} に設定しました。")

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.space:
                if not self.calibration_done:
                    self.calibrate_motors()
                    self.calibration_done = True
                else:
                    self.enable_closed_loop_control()
        except Exception as e:
            self.get_logger().error(f"キー入力処理中にエラーが発生しました: {e}")

    def calibrate_motors(self):
        self.get_logger().info("すべてのモーターをキャリブレーションしています...")
        for serial_number, odrv in self.odrives.items():
            for axis in [odrv.axis0, odrv.axis1]:
                self.get_logger().info(f"シリアル {serial_number} のモーターをキャリブレーション中...")
                axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    def enable_closed_loop_control(self):
        self.get_logger().info("すべてのモーターをクローズドループ制御に設定しています...")
        for serial_number, odrv in self.odrives.items():
            for axis in [odrv.axis0, odrv.axis1]:
                self.get_logger().info(f"シリアル {serial_number} のモーターをクローズドループ制御に設定中...")
                axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def main(args=None):
    rclpy.init(args=args)
    node = Node('argument_parser')
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
