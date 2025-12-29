import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from arm_interfaces.msg import MotorStat, MotorMove


class Motor(Node):

    def __init__(self):
        super().__init__('Motor')
        # self.declare_parameter('id', 1) # will remove this when have launch file for various motors
        #motor_id = self.get_parameter('id').value
        #self.get_logger().info(f"Motor {motor_id} started")
        #self.can_id = 0x00 + motor_id

        self.can_publisher = self.create_publisher(Frame, 'socketcan_bridge/tx', 20)
        self.can_subscriber = self.create_subscription(Frame, 'socketcan_bridge/rx', self.check_can_msg_callback, 20)

        #move_srv_name = 'motor_move_' + str(self.get_parameter('id').value)
        #self.move_service = self.create_service(MotorMove, move_srv_name, self.motor_move_callback) 

        #stat_topic_name = 'motor_stat_' + str(self.get_parameter('id').value)
        self.move_subcriber = self.create_subscription(MotorMove, '/motor_move', self.motor_move_callback, 15) 
        self.stat_publisher = self.create_publisher(MotorStat, '/motor_stat', 15)
        
        timer_period = 5  # seconds
        self.stat_timer = self.create_timer(timer_period, self.stat_timer_callback)

    def check_can_msg_callback(self, can_rx_msg):
        if can_rx_msg.data[0] == 0xA4 or can_rx_msg.data[0] == 0xAE:
            motor_stat = self.status_read(can_rx_msg)
            self.stat_publisher.publish(motor_stat)

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

    def motor_move_callback(self, motor_move_msg):
        motor_id = motor_move_msg.id
        move_can_msg = Frame()
        if motor_move_msg.mode is True:
            motor_count = int(motor_move_msg.angle / 360 * 16384)
            #self.get_logger().info(f"Count is {motor_count}")
            move_can_msg = self.position_control(motor_id, motor_count)
        else:
            motor_spd = int(motor_move_msg.angle * 100)
            #self.get_logger().info(f"Speed is {motor_spd}")
            move_can_msg = self.speed_control(motor_id, motor_spd)
        self.can_publisher.publish(move_can_msg)

    def position_control(self, id, count):
        msg = Frame()
        msg.id = id
        msg.dlc = 0x05
        msg.data[0] = 0xC2
        b = count.to_bytes(4, 'little', signed=True)
        #self.get_logger().info(f"Byte frame is {b[0]}, {b[1]}, {b[2]}, {b[3]}")
        msg.data[1] = b[0]
        msg.data[2] = b[1]
        msg.data[3] = b[2]
        msg.data[4] = b[3]
        return msg
    
    def speed_control(self, id, speed):
        msg = Frame()
        msg.id = id
        msg.dlc = 0x05
        msg.data[0] = 0xC1
        b = speed.to_bytes(4, 'little', signed=True)
        msg.data[1] = b[0]
        msg.data[2] = b[1]
        msg.data[3] = b[2]
        msg.data[4] = b[3]
        return msg

    def status_read(self, can_stat):
        stat = MotorStat()
        stat.id = int(can_stat.id)
        stat.temp = 0
        stat.current = 0.0
        stat.speed = 0.0
        stat.angle = 0.0
        stat.busv = 0.0
        stat.busc = 0.0
        stat.mode = ''
        stat.fault = ''
        if can_stat.data[0] == 0xA4:
            # self.get_logger().info(f"Motor {stat.id} reading status A4 {can_msg_data}")
            stat.temp = int(can_stat.data[1])
            stat.current = float(int.from_bytes(can_stat.data[2:4], 'little', signed=True)) * 0.001
            stat.speed = float(int.from_bytes(can_stat.data[4:6], 'little', signed=True)) * 0.01
            stat.angle = float(int.from_bytes(can_stat.data[6:8], 'little', signed=False)) * (360/16384)
        elif can_stat.data[0] == 0xAE:
            #self.get_logger().info(f"Motor {stat.id} reading status AE {can_msg_data}")
            stat.busv = float(int.from_bytes(can_stat.data[1:3], 'little', signed=False)) * 0.01
            stat.busc = float(int.from_bytes(can_stat.data[3:5], 'little', signed=False)) * 0.01
            stat.temp = int(can_stat.data[5])
            mode = int(can_stat.data[6])
            if mode == 0:
                stat.mode = 'Disabled'
            elif mode == 1:
                stat.mode = 'Voltage Control'
            elif mode == 2:
                stat.mode = 'Current Control'
            elif mode == 3:
                stat.mode = 'Speed Control'
            elif mode == 4:
                stat.mode = 'Position Control'
            else:
                stat.mode = 'No Mode'
            fault = int(can_stat.data[7])
            stat.fault = ''
            if (fault >> 0) & 1:
                stat.fault += 'Voltage '
            if (fault >> 1) & 1:
                stat.fault += 'Current '
            if (fault >> 2) & 1:
                stat.fault += 'Temperature '
            if (fault >> 3) & 1:
                stat.fault += 'Encoder '
            if (fault >> 6) & 1:
                stat.fault += 'Hardware '
            if (fault >> 7) & 1:
                stat.fault += 'Software'
        return stat

def main():
    rclpy.init()
    node = Motor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()