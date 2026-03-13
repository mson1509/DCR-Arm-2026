import rclpy
from rclpy.node import Node

from arm_interfaces.msg import MotorStat1, MotorStat2
from can_msgs.msg import Frame
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

from motor_node.iksolve import IKSolver
from motor_node.motor import Motor

class Controller(Node):

    def __init__(self):
        super().__init__('Controller')
        self.mode = 0 # 0 is fk-spd control, 1 is ik-pos control
        self.last_toggle_buttons = False
        self.get_logger().info(f"Controller started in {self.mode} mode")

        self.motor = Motor()

        self.can_publisher = self.create_publisher(Frame, 'socketcan_bridge/tx', 20)
        self.can_subscriber = self.create_subscription(Frame, 'socketcan_bridge/rx', self.check_can_msg_callback, 20)

        self.ik_solver = IKSolver()
        #self.joints = [0.0, 0.0, 0.0,0.0, 0.0, 0.0]
        self.current_joints = [0.0, 0.0, 0.0]
        self.pos_goal = Point() #for Ik
        self.pos_current = Point()

        #self.motor_move_publisher = self.create_publisher(MotorMove, '/motor_move', 15)
        #self.motor_stat1_subscriber = self.create_subscription(MotorStat1, 'motor_stat_1', self.motor_stat1_callback, 50)
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 30)

        self.stat_publisher_1 = self.create_publisher(MotorStat1, '/motor_stat_1', 15)
        self.stat_publisher_2 = self.create_publisher(MotorStat2, '/motor_stat_2', 15)
        timer_period = 1  # seconds
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
                self.get_logger().info(f"Motor {i + 1} going to {angle_cmd[i]}")
                can_cmd = self.motor.position_control(i + 1, angle_cmd[i])
                self.get_logger().info(f"Byte frame is {can_cmd.data[1]}, {can_cmd.data[2]}, {can_cmd.data[3]}, {can_cmd.data[4]}")
                self.can_publisher.publish(can_cmd) 
        elif (self.mode == 0):
            spd_cmd[0] = joy_msg.axes[0] * 5
            spd_cmd[1] = joy_msg.axes[1] * 5
            spd_cmd[2] = joy_msg.axes[4] * 5
            for i in range(3):
                can_cmd = self.motor.speed_control(i + 1, spd_cmd[i])
                self.can_publisher.publish(can_cmd)
        
        spd_cmd[3] = joy_msg.axes[3] * 10 # Rx
        spd_cmd[4] = joy_msg.axes[7] * 10 # Pad y
        spd_cmd[5] = joy_msg.axes[6] * 10 # Pad x
        for i in range (3,6):
            can_cmd = self.motor.speed_control(i + 1, spd_cmd[i])
            self.can_publisher.publish(can_cmd)
        
        #self.publish_motor_move()
        self.pos_current = self.ik_solver.desolve(self.current_joints)

        # if (joy_msg.buttons[0] == 1):
        #     if (self.mode == 1):
        #         self.get_logger().info(f"going to x:{(self.pos_goal.x * 1000):.2f} and y:{(self.pos_goal.y * 1000):.2f} and z:{(self.pos_goal.z * 1000):.2f}")
        #         self.get_logger().info("sending joints: [" + ", ".join(f"{j:.2f}" for j in self.joints) + "]")            
        #     self.get_logger().info(f"currently at x:{(self.pos_current.x * 1000):.2f} and y:{(self.pos_current.y * 1000):.2f} and z:{(self.pos_current.z * 1000):.2f}")
        #     self.get_logger().info("joints currenly is: [" + ", ".join(f"{j:.2f}" for j in self.current_joints  ) + "]")  
            
    def check_can_msg_callback(self, can_rx_msg):
        if can_rx_msg.data[0] == 0xA4:
            motor_stat = self.motor.read_status_1(can_rx_msg)
            if (motor_stat.id < 4):
                self.current_joints[motor_stat.id - 1] = motor_stat.angle
            self.stat_publisher_1.publish(motor_stat)
        elif can_rx_msg.data[0] == 0xAE:
            motor_stat = self.motor.read_status_2(can_rx_msg)
            self.stat_publisher_2.publish(motor_stat)

    def stat_timer_callback(self):
        stat_msg1 = Frame()
        stat_msg1.id = 0xFF
        stat_msg1.dlc = 0x01
        stat_msg1.data[0] = 0xA4
        self.can_publisher.publish(stat_msg1)

        stat_msg2 = Frame()
        stat_msg2.id = 0xFF
        stat_msg2.dlc = 0x01
        stat_msg2.data[0] = 0xAE
        self.can_publisher.publish(stat_msg2)      

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