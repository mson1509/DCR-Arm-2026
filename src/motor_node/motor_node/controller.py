import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from arm_interfaces.msg import MotorStat1, MotorStat2
from can_msgs.msg import Frame
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

from motor_node.iksolve import IKSolver
from motor_node.motor import Motor

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')

        self.declare_parameter('fk_speed', [5, 5, 5, 10, 10, 10])
        self.fk_speed = self.get_parameter('fk_speed').value
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.mode = 0 # 0 is fk-spd control, 1 is ik-pos control
        self.last_toggle_buttons = False
        self.get_logger().info(f"Controller started in {self.mode} mode")

        self.motor = Motor()

        self.can_publisher = self.create_publisher(Frame, 'socketcan_bridge/tx', 20)
        self.can_subscriber = self.create_subscription(Frame, 'socketcan_bridge/rx', self.check_can_msg_callback, 20)

        self.estop_subscriber = self.create_subscription(Bool, '/estop', self.estop_callback, 10)

        self.ik_solver = IKSolver()
        #self.joints = [0.0, 0.0, 0.0,0.0, 0.0, 0.0]
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pos_goal = Point() #for Ik
        self.pos_current = Point()

        self.ee_pos = 137 #137 is fully open, 105 is fully closed
        self.last_ee_pos = 137
        self.last_laser_button = False

        self.joint_limits = [
            {"min": -180.0, "max": 180.0},  # Joint 1
            {"min":  0, "max":  180.0},  # Joint 2
            {"min": 0, "max": 270.0},  # Joint 3
            {"min":  -90.0, "max":  90.0},  # Joint 4
            {"min": -90.0, "max": 90.0},  # Joint 5
            {"min":  -180.0, "max":  180.0},  # Joint 6
        ]
        #self.motor_move_publisher = self.create_publisher(MotorMove, '/motor_move', 15)
        #self.motor_stat1_subscriber = self.create_subscription(MotorStat1, 'motor_stat_1', self.motor_stat1_callback, 50)
        self.joy_subscriber = self.create_subscription(Joy, '/arm/joy', self.joy_callback, 30)
        self.viz_publisher = self.create_publisher(JointState, '/joint_states', 15)

        self.stat_publisher_1 = self.create_publisher(MotorStat1, '/motor_stat_1', 15)
        self.stat_publisher_2 = self.create_publisher(MotorStat2, '/motor_stat_2', 15)
        timer_period = 0.5  # seconds
        self.stat_timer = self.create_timer(timer_period, self.stat_timer_callback)

        for i in range (6):
            can_cmd = self.motor.set_home(i + 1)
            self.can_publisher.publish(can_cmd)

    def joy_callback(self, joy_msg):
        toggle_pressed = (joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 1)
        if toggle_pressed and not self.last_toggle_buttons:
            if self.mode == 0:
                self.pos_goal = self.pos_current
                self.ik_solver.old_joints = [0.0] + [j / 57.2958 for j in self.current_joints] + [0.0]
                #self.get_logger().info(f"new old joints are: {str(self.ik_solver.old_joints)}")
                self.mode = 1
                self.get_logger().info("going in IK mode")
            elif self.mode == 1:
                self.mode = 0
                self.get_logger().info("going in FK mode")
        self.last_toggle_buttons = toggle_pressed

        spd_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        if (self.mode == 1):
            self.get_logger().info(f"Pos Goal is {self.pos_goal.x}, {self.pos_goal.y}, {self.pos_goal.z}")
            self.pos_goal.x += joy_msg.axes[0] / 500 # Lx
            self.pos_goal.y += joy_msg.axes[1] / 500 # Ly
            self.pos_goal.z += joy_msg.axes[4] / 500 # Ry
            angle_cmd = self.ik_solver.solve(self.pos_goal)
            for i in range(3):
                if (angle_cmd[i] < self.joint_limits[i]["min"] or angle_cmd[i] > self.joint_limits[i]["max"]):
                    self.get_logger().warn(f"Motor {i + 1} reached limit")
                    self.mode = 0
                    self.get_logger().info("Joint limit hit, switching to FK mode")
                    break
                else:
                    self.get_logger().info(f"Motor {i + 1} going to {angle_cmd[i]}")
                    can_cmd = self.motor.position_control(i + 1, angle_cmd[i])
                    #self.get_logger().info(f"Byte frame is {can_cmd.data[1]}, {can_cmd.data[2]}, {can_cmd.data[3]}, {can_cmd.data[4]}")
                    self.can_publisher.publish(can_cmd) 
        elif (self.mode == 0):
            spd_cmd[0] = joy_msg.axes[0] * -self.fk_speed[0] #Lx
            spd_cmd[1] = joy_msg.axes[1] * self.fk_speed[1] #Ly
            spd_cmd[2] = joy_msg.axes[4] * self.fk_speed[2] #Ry
            for i in range(3):
                if (self.current_joints[i] <= self.joint_limits[i]["min"] + 5 and spd_cmd[i] < 0):
                    self.get_logger().warn(f"Motor {i + 1} reaching min limit")
                    spd_cmd[i] = 0
                elif (self.current_joints[i] >= self.joint_limits[i]["max"] - 5 and spd_cmd[i] > 0):
                    self.get_logger().warn(f"Motor {i + 1} reaching max limit")
                    spd_cmd[i] = 0
                can_cmd = self.motor.speed_control(i + 1, spd_cmd[i])
                self.can_publisher.publish(can_cmd)
        
        spd_cmd[3] = joy_msg.axes[3] * -self.fk_speed[3] # Rx
        spd_cmd[4] = joy_msg.axes[7] * -self.fk_speed[4] # Pad y
        spd_cmd[5] = joy_msg.axes[6] * self.fk_speed[5] # Pad x
        for i in range (3,6):
            if (self.current_joints[i] <= self.joint_limits[i]["min"] + 5 and spd_cmd[i] < 0):
                    self.get_logger().warn(f"Motor {i + 1} reaching min limit")
                    spd_cmd[i] = 0
            elif (self.current_joints[i] >= self.joint_limits[i]["max"] - 5 and spd_cmd[i] > 0):
                self.get_logger().warn(f"Motor {i + 1} reaching max limit")
                spd_cmd[i] = 0
            can_cmd = self.motor.speed_control(i + 1, spd_cmd[i])
            self.can_publisher.publish(can_cmd)
        
        if (joy_msg.buttons[1] == 1):
            self.ee_pos = self.ee_pos + 2 # open with btn O
        if (joy_msg.buttons[3] == 1):
            self.ee_pos = self.ee_pos - 2 # close with btn SQUARE
        self.ee_pos = max(105, min(self.ee_pos, 137))
        if (abs(self.ee_pos - self.last_ee_pos) >= 4):
            self.last_ee_pos = self.ee_pos
            ee_cmd = self.motor.ee_set_pos(self.ee_pos)
            self.can_publisher.publish(ee_cmd)
        if (joy_msg.buttons[0] == 1 and not self.last_laser_button):
            self.can_publisher.publish(self.motor.ee_laser())
        self.last_laser_button = (joy_msg.buttons[0] == 1) # laser with btn X

        if (joy_msg.buttons[10] == 1):
             self.can_publisher.publish(self.motor.clr_faults()) # clear faults with PS

        self.pos_current = self.ik_solver.desolve(self.current_joints)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'fk_speed':
                self.fk_speed = param.value
                self.get_logger().info(f"fk_speed updated to {self.fk_speed}")
        return SetParametersResult(successful=True)
    
    def estop_callback(self, estop):
        if (estop.data):
            if (self.mode == 1):
                self.mode = 0
                self.get_logger().info("Lost comms, switching to FK mode")
            for i in range(6):
                can_cmd = self.motor.speed_control(i + 1, 0)
                self.can_publisher.publish(can_cmd)

        # fk mode
        # set spd_cmd = 0
        

        # if (joy_msg.buttons[0] == 1):
        #     if (self.mode == 1):
        #         self.get_logger().info(f"going to x:{(self.pos_goal.x * 1000):.2f} and y:{(self.pos_goal.y * 1000):.2f} and z:{(self.pos_goal.z * 1000):.2f}")
        #         self.get_logger().info("sending joints: [" + ", ".join(f"{j:.2f}" for j in self.joints) + "]")            
        #     self.get_logger().info(f"currently at x:{(self.pos_current.x * 1000):.2f} and y:{(self.pos_current.y * 1000):.2f} and z:{(self.pos_current.z * 1000):.2f}")
        #     self.get_logger().info("joints currenly is: [" + ", ".join(f"{j:.2f}" for j in self.current_joints  ) + "]")  
            
    def check_can_msg_callback(self, can_rx_msg):
        if can_rx_msg.data[0] == 0xA4:
            motor_stat = self.motor.read_status_1(can_rx_msg)
            #if (motor_stat.id < 4):
            if 1 <= motor_stat.id <= 6:
                self.current_joints[motor_stat.id - 1] = motor_stat.angle
            self.stat_publisher_1.publish(motor_stat)
        elif can_rx_msg.data[0] == 0xAE:
            motor_stat = self.motor.read_status_2(can_rx_msg)
            self.stat_publisher_2.publish(motor_stat)
       # if can_rx_msg.id == 0x107:
            

    def stat_timer_callback(self):
        stat_msg1 = self.motor.send_status_1()
        self.can_publisher.publish(stat_msg1)

        stat_msg2 = self.motor.send_status_2()
        self.can_publisher.publish(stat_msg2)  

        viz_msg = JointState()
        viz_msg.header.stamp = self.get_clock().now().to_msg()
        viz_msg.name = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
        ]
        viz_msg.position = [j * 0.0174533 for j in self.current_joints[:3]]  # only first 3, degrees to radians
        self.viz_publisher.publish(viz_msg)

    # def motor_stat1_callback(self, stat):
    #     if (stat.id == 1):
    #         self.current_joints[0] = stat.angle
    #     elif (stat.id == 2):
    #         self.current_joints[1] = stat.angle
    #     elif (stat.id == 3):
    #         self.current_joints[2] = stat.angle

    # def publish_motor_move(self):
    #     for i in range (6):
    #         command = MotorMove()
    #         command.id = i + 1
    #         command.angle = self.joints[i]
    #         if (self.mode == 1 and i < 3):
    #             command.mode = True
    #         else:
    #             #self.get_logger().info(f"Controller sending fk mode")
    #             command.mode = False
    #         self.motor_move_publisher.publish(command)

def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

        # dx = joy_msg.axes[0]
        # dy = joy_msg.axes[1]
        # dz = joy_msg.axes[2]

        # if dx != 0 or dy != 0 or dz != 0:
        #     self.pos[0] += dx
        #     self.pos[1] += dy
        #     self.pos[2] += dz

        #     # Solve IK
        #     #joints = IkSolver(self.pos)   # return [j1, j2, j3]

        #     # Send to first 3 motors
        #     for i in range(3):
        #         req = MotorMove.Request()
        #         req.angle = self.pos[i] #temp
        #         self.get_logger().info(f"sending motor {i + 1} angle {self.pos[i]}")
        #         self.move_clients[i].call_async(req)

        # # ---------------------------
        # #  2) ORIENTATION (buttons)
        # # ---------------------------
        # orientation_buttons = [
        #     (0, 1),  # yaw:  +btn0, -btn1
        #     (2, 3),  # pitch: +btn2, -btn3
        #     (4, 5)   # roll: +btn4, -btn5
        # ]

        # for j, (btn_plus, btn_minus) in enumerate(orientation_buttons):
        #     delta = joy_msg.buttons[btn_plus] - joy_msg.buttons[btn_minus]
        #     if delta != 0:
        #         self.orientation[j] += delta
        #         req = MotorMove.Request()
        #         req.angle = self.orientation[j]
        #         self.get_logger().info(f"sending motor {j + 4} angle {self.orientation[j]}")
        #         self.move_clients[j+3].call_async(req)  