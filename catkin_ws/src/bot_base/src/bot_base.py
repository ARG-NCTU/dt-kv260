#!/usr/bin/env python3

from time import sleep, time
from pynq_dpu import DpuOverlay
# Load DPU uart overlay
overlay = DpuOverlay("/home/argnctu/dt-kv260/overlays/dpu_uartlite/dpu_uartlite.bit", download=False)
if not overlay.is_loaded():
    start = time()
    overlay.download()
    end = time()
    print("Load overlay using {}ms.".format((end-start)))

import rospy
from geometry_msgs.msg import Twist

RX_FIFO = 0x00
TX_FIFO = 0x04

#Status Reg
STAT_REG = 0x08
RX_VALID = 0
RX_FULL = 1
TX_EMPTY = 2
TX_FULL = 3
IS_INTR = 4
OVERRUN_ERR = 5
FRAME_ERR = 6
PARITY_ERR =7

#Ctrl Reg
CTRL_REG = 0x0C
RST_TX = 0
RST_RX = 1
INTR_EN = 4

class UartLite:
    def __init__(self, uart):
        self.uart = uart
        self.setupCtrlReg()

    def getBit(self,num,pos):
        return (num&1<<pos)>>pos

    def setupCtrlReg(self):
        # Reset FIFOs, disable interrupts
        self.uart.write(CTRL_REG, 1<<RST_TX | 1<<RST_RX)
        sleep(0.5)
        self.uart.write(CTRL_REG, 0)
        sleep(0.5)

    def status(self):
        """Returns object that specifies current status of axi core"""
        status = self.uart.read(STAT_REG)
        return {'RX_VALID':self.getBit(status,RX_VALID),
            'RX_FULL':self.getBit(status, RX_FULL),
            'TX_EMPTY':self.getBit(status, TX_EMPTY),
            'TX_FULL':self.getBit(status, TX_FULL),
            'IS_INTR':self.getBit(status, IS_INTR),
            'OVERRUN_ERR':self.getBit(status, OVERRUN_ERR),
            'FRAME_ERR':self.getBit(status, FRAME_ERR),
            'PARITY_ERR':self.getBit(status, PARITY_ERR)}

    def read(self, count, timeout=10):
        buf = ""
        stop_time = time() + timeout
        for i in range(count):
            while (not (self.uart.read(STAT_REG) & 1<<RX_VALID)) and (time()<stop_time):
                pass
            if time() >= stop_time:
                break
            buf += chr(self.uart.read(RX_FIFO))
        return buf

    def write(self, buf, timeout=10):
        stop_time = time() + timeout
        wr_count = 0
        for i in buf:
            while (self.uart.read(STAT_REG) & 1<<TX_FULL) and (time()<stop_time):
                pass
            if time() > stop_time:
                break
            self.uart.write(TX_FIFO, ord(i))
            wr_count += 1
        return wr_count

    def readline(self):
        ret = ""
        buf = self.read(1)
        if len(buf) == 0:
            return ""
        while buf != '\n':
            ret += buf
            buf = self.read(1)
        return ret

# Setup uart
uart = UartLite(overlay.axi_uartlite_0)

class Motor_control():
    def __init__(self):
        self.linear_x = 0
        self.angular_z = 0

        # Subscriber
        self.sub_cmd = rospy.Subscriber("cmd_vel", Twist, self.cb_cmd_vel)
        rospy.Timer(rospy.Duration(0.2), self.cmd_vel_control)


    def cb_cmd_vel(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def cmd_vel_control(self,event):
        linear_x, angular_z = self.linear_x,self.angular_z
        print("linear_x, angular_z ",linear_x, angular_z)

        if (linear_x==0 and angular_z==0):
            speed_wish_right, speed_wish_left = 0, 0
        else:
            # tmp_r = (angular_z*0.5)/2 + linear_x
            # tmp_l = linear_x*2 - tmp_r
            tmp_r =linear_x + angular_z
            tmp_l = linear_x - angular_z
            # float WHEEL_DIST = 5;
            # float speed_wish_right = ((cmd_vel.angular.z * WHEEL_DIST)/2 + cmd_vel.linear.x);
            # float speed_wish_left = (cmd_vel.linear.x * 2-speed_wish_right);


            if tmp_r > 0: speed_wish_right = int(125 + tmp_r*75)
            else: speed_wish_right = int(-125 + tmp_r*75)

            if tmp_l > 0: speed_wish_left = int(125 + tmp_l*75)
            else: speed_wish_left = int(-125 + tmp_l*75)

            if speed_wish_right > 255: speed_wish_right = 255
            if speed_wish_right < -255: speed_wish_right = -255
            if speed_wish_left > 255: speed_wish_left = 255
            if speed_wish_left < -255: speed_wish_left = -255

        # print("speed_wish_right, speed_wish_left= ",speed_wish_right, ",",speed_wish_left)

        pkg="1" if speed_wish_right>=0 else "0"
        speed_wish_right = abs(speed_wish_right)
        pkg += str(speed_wish_right//100)
        pkg += str((speed_wish_right%100)//10)
        pkg += str(speed_wish_right%10)

        if speed_wish_left>=0: pkg += "1"
        else : pkg += "0"
        speed_wish_left = abs(speed_wish_left)
        pkg += str(speed_wish_left//100)
        pkg += str((speed_wish_left%100)//10)
        pkg += str(speed_wish_left%10)
        pkg += "\n"

        # print("uart pkg= ", pkg)

        uart.write(pkg)


if __name__=="__main__":
    rospy.init_node("bot_base", anonymous=True)
    motor_control = Motor_control()
    rospy.spin()
