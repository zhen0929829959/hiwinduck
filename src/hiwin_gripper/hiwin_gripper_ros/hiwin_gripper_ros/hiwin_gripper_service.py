from ctypes import CDLL
import time
import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from hiwin_gripper_interfaces.srv import GripperCommand


file_directory = os.path.realpath(__file__)
file_directory = os.path.abspath(os.path.join(file_directory, '../../..'))
so_file = file_directory + "/HIWIN_XEG32/src/controller/Hiwin_API.so"
modbus = CDLL(so_file)


class HiwinGripperService(Node):
    def __init__(self):
        super(HiwinGripperService, self).__init__('hiwin_gripper_service')
        self.srv = self.create_service(GripperCommand, 'execute_gripper', self.execute_gripper_callback)
        self.gripper_state = 1

    def execute_gripper_callback(self, request, response):
        if request.cmd_mode==1:
            self.on()
        elif request.cmd_mode==2:
            self.reset()
        elif request.cmd_mode==3:
            self.move(3200,4800,1)
        elif request.cmd_mode==4:
            self.move(100,4800,1)
        elif request.cmd_mode==5:
            self.move(request.distance, request.speed, request.flag)
        elif request.cmd_mode==6:
            print(request)
            self.expert(request.direction, request.distance, 
                        request.speed,request.holding_stroke, 
                        request.holding_speed, request.holding_force,request.flag)
        elif request.cmd_mode==7:
            self.off()
        response.gripper_state = int(self.gripper_state)
        return response

    #move
    #功能 : 絕對位置移動模式
    #參數 : 
    #    dis = 位置(0~3200)
    #    speed = 速度(0~6000)
    #    flag = 旗標1(待確定)

    def on(self):
        modbus.libModbus_Connect(1)
            
        self.get_logger().info("modbus on")
        return

    def reset(self):
        modbus.reset()
        time.sleep(25)
        self.get_logger().info("gripper reset")
        return

    def move(self, dis, speed, flag):
        self.gripper_state = 1
        modbus.move(dis, speed, flag)
        time.sleep(0.01)
        state = self.gripper_state
        while state!='2':
            state = str(modbus.read_mode())
            self.gripper_state = state
            if state=='3'or state =='0':
                break
        self.get_logger().info("end move")
        return

    def expert(self, dir, dis, speed, Holding_stroke, Holding_speed , Holding_force, flag):
        self.gripper_state = 1
        modbus.expert(dir, dis, speed, Holding_stroke, Holding_speed , Holding_force, flag)
        time.sleep(0.01)
        state = self.gripper_state
        while state!='3':
            state = str(modbus.read_mode())
            self.gripper_state = state
            if state=='0':
                break
        self.get_logger().info("end expert")
        return

    def off(self):
        modbus.Modbus_Close()
        self.get_logger().info("modbus off")
        return


def main():
    rclpy.init()

    gripper_service = HiwinGripperService()

    gripper_service.on()
    gripper_service.reset()
    # gripper_service.expert(0, 600,6000,2600,500,40,1)

    rclpy.spin(gripper_service)

    rclpy.shutdown()

if __name__ == "__main__":

    main()
