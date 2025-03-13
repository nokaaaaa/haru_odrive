import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, Float32
import odrive
import json
import math
from sensor_msgs.msg import Joy

class ODriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        self.declare_parameter("config_file", "config.json")
        config_path = self.get_parameter("config_file").value
        self.get_logger().info(f'Loading config from {config_path}')
        
        with open(config_path, 'r') as file:
            self.config = json.load(file)
        
        self.R = self.config.get("R", 0.2)
        self.wheel_radius = self.config.get("wheel_radius", 0.05)
        
        self.odrives = {}
        self.load_odrives()
        
        self.create_subscription(Twist, '/pid_cmd_vel', self.twist_callback, 10)
        self.create_subscription(Pose2D,'/pose',self.pose_callback, 10)
        self.create_subscription(Bool, '/calib', self.calib_callback, 10)
        self.create_subscription(Bool, '/closed', self.closed_loop_callback, 10)

        self.theta = 0

        self.encoder_publisher_l = self.create_publisher(Float32, '/motor_encoder_position_l', 10)
        self.encoder_publisher_r = self.create_publisher(Float32, '/motor_encoder_position_r', 10)

        # 定期的にエンコーダの位置をパブリッシュ
        self.create_timer(0.0001, self.encoder_position_publish_callback)  # 0.0001秒毎にエンコーダ位置をパブリッシュ
        self.create_subscription(Bool,'reset_pos',self.reset_encoder_position,10)
        for motor in self.config["motors"]:
            self.create_subscription(Float32, motor["topic_name"], self.create_motor_callback(motor), 10)
    
    
    def reset_encoder_position(self , msg):
       self.get_logger().info(f"Received reset command: {msg.data}")
       if msg.data == True: 
          for serial, odrv in self.odrives.items():
            if odrv:
                 odrv.axis1.encoder.set_linear_count(0)
                 odrv.axis0.encoder.set_linear_count(0)
                 self.get_logger().info(f"Reset encoder position for ODrive {serial}")
            else:
                 self.get_logger().error(f"ODrive {serial} not found!")

    def load_odrives(self):
        for motor in self.config["motors"]:
            serial = motor["serial_number"]
            if serial not in self.odrives:
                self.get_logger().info(f'Connecting to ODrive {serial}...')
                self.odrives[serial] = odrive.find_any(serial_number=serial)
                self.get_logger().info(f'Connected to ODrive {serial}')

    def encoder_position_publish_callback(self):
        # config.jsonの5番目と6番目のモーターのみを処理
        motors_to_check = [4,5]  # インデックスは0から始まるので、5番目と6番目はインデックス4と5
        for index in motors_to_check:
            if index < len(self.config["motors"]):
                motor = self.config["motors"][index]
                serial = motor["serial_number"]
                odrv = self.odrives.get(serial)
                if odrv:
                    # アクシス0とアクシス1のエンコーダ位置を取得
                    axis0_position = odrv.axis0.encoder.pos_estimate
                    axis1_position = odrv.axis1.encoder.pos_estimate
                    
                    # それぞれのエンコーダ位置をパブリッシュ
                    self.encoder_publisher_r.publish(Float32(data=axis0_position))
                    self.encoder_publisher_l.publish(Float32(data=axis1_position))
                    self.get_logger().info(f"Published encoder position for motor {serial}:  axis0={axis0_position}, axis1={axis1_position}")
                else:
                    self.get_logger().error(f'ODrive {serial} not found!')  

    def pose_callback(self,msg):
        self.yaw = msg.theta          
    
    def twist_callback(self, msg):
        Vx = msg.linear.x
        Vy = msg.linear.y
        omega = msg.angular.z
        
        coeff = 1 / (2 * math.pi * self.wheel_radius)
        
        v1 =coeff*( -math.sin(self.yaw + math.pi / 4) * Vx + math.cos(self.yaw + math.pi / 4) * Vy + self.R * omega)
        v2 = coeff*(-math.sin(self.yaw + math.pi * 3 / 4) * Vx + math.cos(self.yaw + math.pi * 3 / 4) * Vy + self.R * omega)
        v3 = coeff*(-math.sin(self.yaw + math.pi * 5 / 4) * Vx + math.cos(self.yaw + math.pi * 5 / 4) * Vy + self.R * omega)
        v4 =coeff*( -math.sin(self.yaw + math.pi * 7 / 4) * Vx + math.cos(self.yaw + math.pi * 7 / 4) * Vy + self.R * omega)
        
        wheel_speeds = [v1, v2, v3, v4]
        
        if len(self.config["motors"]) < 4:
            self.get_logger().error('Not enough motors configured for twist command')
            return
        
        for i, motor in enumerate(self.config["motors"][:4]):
            odrv = self.odrives.get(motor["serial_number"])
            if odrv:
                axis = odrv.axis0 if motor["axis"] == 0 else odrv.axis1
                if axis.controller.config.control_mode != 2:
                    self.get_logger().warn(f'Switching {motor["serial_number"]} axis {motor["axis"]} to velocity control mode')
                    axis.controller.config.control_mode = 2  
                axis.controller.input_vel = wheel_speeds[i]
            else:
                self.get_logger().error(f'ODrive {motor["serial_number"]} not found!')
    
    def create_motor_callback(self, motor):
        def motor_callback(msg):
            odrv = self.odrives.get(motor["serial_number"])
            if odrv:
                axis = odrv.axis0 if motor["axis"] == 0 else odrv.axis1
                axis.controller.input_vel = msg.data
            else:
                self.get_logger().error(f'ODrive {motor["serial_number"]} not found!')
        return motor_callback

    # def listener_callback(self,msg):
    #     if msg.buttons[10] == 1:
    #         for odrv in self.odrives.values():
    #             odrv.config.dc_max_negative_current = -20
                # odrv.axis0.motor.config.torque_constant = 8.27/270
                # odrv.axis1.motor.config.torque_constant = 8.27/270
                # odrv.axis0.controller.config.vel_limit = 108
                # odrv.axis1.controller.config.vel_limit = 108
                # odrv.axis0.motor.config.resistance_calib_max_voltage = 10
                # odrv.axis1.motor.config.resistance_calib_max_voltage = 10

         
    def calib_callback(self, msg):
        if msg.data:
            self.get_logger().info('Calibrating all motors...')
            for odrv in self.odrives.values():
                odrv.axis0.requested_state = 3  # AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                odrv.axis1.requested_state = 3
    
    def closed_loop_callback(self, msg):
        if msg.data:
            self.get_logger().info('Starting closed-loop control...')
            for odrv in self.odrives.values():
                odrv.axis0.controller.config.control_mode = 2
                odrv.axis1.controller.config.control_mode = 2
                odrv.axis0.requested_state = 8  # AXIS_STATE_CLOSED_LOOP_CONTROL
                odrv.axis1.requested_state = 8

def main(args=None):
    rclpy.init(args=args)
    node = ODriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
