#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

from pty import slave_open
from typing import overload
import rospy

import sys
import struct
import time
import threading
import ctypes
from collections import namedtuple
import pysoem
import os
import numpy as np

# import geometry_msgs.msg
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Wrench, WrenchStamped
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

from rft64_6a01_ros_interface.srv import set_LPF_rate, set_LPF_rateResponse

class RFT64:
    BECKHOFF_VENDOR_ID = 0x000008EE
    TFSENSOR_PRODUCT_CODE = 0x00000003

    def set_ft_rate(self, req):
        rate = req.rate
        self.slave.sdo_write(0x2001, 2, struct.pack('h', rate.data))
        p_rate = int.from_bytes(self.slave.sdo_read(0x2001, 2), byteorder='little', signed=False)
        res = set_LPF_rateResponse()
        res.rate.data = p_rate
        return res

    def __init__(self, ifname):
        rospy.init_node('RFT64', anonymous=True)
        self.rate = rospy.Rate(500)

        self._ifname = ifname
        self._pd_thread_stop_event = threading.Event()
        self._ch_thread_stop_event = threading.Event()
        self._actual_wkc = 0
        self._master = pysoem.Master()
        self._master.in_op = False
        self._master.do_check_state = False
        SlaveSet = namedtuple('SlaveSet', 'name product_code config_func')
        self._expected_slave_layout = {0: SlaveSet('RFT64', self.TFSENSOR_PRODUCT_CODE, self.tfsensor_setup),
                                     1: SlaveSet('RFT64', self.TFSENSOR_PRODUCT_CODE, self.tfsensor_setup)}
        self.collapse_time = 0.0

        #ROS publisher
        self.Model_name_pub = rospy.Publisher("RFT64/Model_name", String, queue_size=1)
        self.Serial_number_pub = rospy.Publisher("RFT64/Serial_number", String, queue_size=1)
        self.Firmware_version_pub = rospy.Publisher("RFT64/Firmware_version", String, queue_size=1)

        self.Bias_pub = rospy.Publisher("RFT64/Bias", Int16, queue_size=1)
        self.LPF_setup_pub = rospy.Publisher("RFT64/LPF_setup", Int16, queue_size=1)

        # self.Overload_count_pub = rospy.Publisher("/RFT64/Overload_count", Int16MultiArray, queue_size=1)
        self.Overload_count_pub = rospy.Publisher("RFT64/Overload_count", Wrench, queue_size=1)

        # self.FT_data_pub = rospy.Publisher("/RFT64/FT_data", TwistStamped, queue_size=1)
        # self.FTS_status_pub = rospy.Publisher("/RFT64/FTS_status", TwistStamped, queue_size=1)
        self.FT_data_pub = rospy.Publisher("RFT64/FT_data", WrenchStamped, queue_size=1)
        self.FTS_status_pub = rospy.Publisher("RFT64/FTS_status", WrenchStamped, queue_size=1)
        self.Temperature_pub = rospy.Publisher("RFT64/Temperature", Float64, queue_size=1)

        self.set_LPF_server = rospy.Service("set_FT_LPF", set_LPF_rate, self.set_ft_rate)

        # self._master = pysoem.Master()
        self._master.open(self._ifname)

        if not self._master.config_init() > 0:
            self._master.close()
            raise InterfaceError('no slave found')
        
        for i, slave in enumerate(self._master.slaves):
            if not ((slave.man == self.BECKHOFF_VENDOR_ID) and
                    (slave.id == self._expected_slave_layout[i].product_code)):
                self._master.close()
                raise InterfaceError('unexpected slave layout')
            slave.config_func = self._expected_slave_layout[i].config_func
            slave.is_lost = False

        self._master.config_map()

        print('RFT64 interface started')

    def tfsensor_setup(self, slave_pos):
        print("Initialize sensor...")
        slave = self._master.slaves[slave_pos]
        self.slave = slave

        ### Sensor Information ###
        self.Model_name = slave.sdo_read(0x2000, 1).decode('utf-8')
        Serial_number = slave.sdo_read(0x2000, 2).decode('utf-8')
        Firmware_version = slave.sdo_read(0x2000, 3).decode('utf-8')
        # publish model name
        model_name_msg = String()
        model_name_msg.data = self.Model_name
        self.Model_name_pub.publish(model_name_msg)
        #publish serial number
        serial_number_msg = String()
        serial_number_msg.data = Serial_number
        self.Serial_number_pub.publish(serial_number_msg)
        # publish firmware version
        firmware_version_msg = String()
        firmware_version_msg.data = Firmware_version
        self.Firmware_version_pub.publish(firmware_version_msg)


        ### Sensor Setup ###
        BIAS = int.from_bytes(slave.sdo_read(0x2001, 1), byteorder='little', signed=False)
        LPF_setup = int.from_bytes(slave.sdo_read(0x2001, 2), byteorder='little', signed=False)
        # publish BIAS
        BIAS_msg = Int16()
        BIAS_msg.data = BIAS
        self.Bias_pub.publish(BIAS_msg)
        # publish LPF_Setup
        LPF_setup_msg = Int16()
        LPF_setup_msg.data = LPF_setup
        self.LPF_setup_pub.publish(LPF_setup_msg)


        ### Overload Counter ###
        overload_count_Fx = int.from_bytes(slave.sdo_read(0x2002, 1), byteorder='little', signed=False)
        overload_count_Fy = int.from_bytes(slave.sdo_read(0x2002, 2), byteorder='little', signed=False)
        overload_count_Fz = int.from_bytes(slave.sdo_read(0x2002, 3), byteorder='little', signed=False)
        overload_count_Tx = int.from_bytes(slave.sdo_read(0x2002, 4), byteorder='little', signed=False)
        overload_count_Ty = int.from_bytes(slave.sdo_read(0x2002, 5), byteorder='little', signed=False)
        overload_count_Tz = int.from_bytes(slave.sdo_read(0x2002, 6), byteorder='little', signed=False)
        # publish Overload count 
        # overload_count_msg = Int16MultiArray()
        overload_count_msg = Wrench
        # overload_count_msg.data = [overload_count_Fx, overload_count_Fy, overload_count_Fz, overload_count_Tx, overload_count_Ty, overload_count_Tz]
        # overload_count_msg.force.x = overload_count_Fx
        # overload_count_msg.force.y = overload_count_Fy
        # overload_count_msg.force.z = overload_count_Fz
        # overload_count_msg.torque.x = overload_count_Tx
        # overload_count_msg.torque.y = overload_count_Ty
        # overload_count_msg.torque.z = overload_count_Tz
        # self.Overload_count_pub.publish(overload_count_msg)


        # ### TxPDO ###
        # TxPDO_Fx = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 1))
        # TxPDO_Fy = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 2))
        # TxPDO_Fz = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 3))
        # TxPDO_Tx = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 4))
        # TxPDO_Ty = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 5))
        # TxPDO_Tz = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 6))
        # TxPDO_FTS_Status = int.from_bytes(slave.sdo_read(0x6000, 7), byteorder='little', signed=False)
        # TxPDO_Temperature = ctypes.c_float.from_buffer_copy(slave.sdo_read(0x6000, 8))
        # # publish FT data
        # tf_data_msg = TwistStamped()
        # tf_data_msg.header.stamp = rospy.Time.now()
        # tf_data_msg.header.frame_id = 'RFT64-6A01'
        # tf_data_msg.twist.linear.x = np.float64((TxPDO_Fx).value)
        # tf_data_msg.twist.linear.y = np.float64((TxPDO_Fy).value)
        # tf_data_msg.twist.linear.z = np.float64((TxPDO_Fz).value)
        # tf_data_msg.twist.angular.x = np.float64((TxPDO_Tx).value)
        # tf_data_msg.twist.angular.y = np.float64((TxPDO_Ty).value)
        # tf_data_msg.twist.angular.z = np.float64((TxPDO_Tz).value)
        # self.FT_data_pub.publish(tf_data_msg)
        # # publish FTS-Status
        # FTS_status_msg = TwistStamped()
        # FTS_status_msg.header = tf_data_msg.header
        # share = 0
        # if (TxPDO_FTS_Status % 2 == 0):
        #     FTS_status_msg.twist.angular.z = 0
        # else:
        #     FTS_status_msg.twist.angular.z = 1
        # share = TxPDO_FTS_Status // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.angular.y = 0
        # else:
        #     FTS_status_msg.twist.angular.y = 1
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.angular.x = 0
        # else:
        #     FTS_status_msg.twist.angular.x = 1
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.linear.z = 0
        # else:
        #     FTS_status_msg.twist.linear.z = 1   
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.linear.y = 0
        # else:
        #     FTS_status_msg.twist.linear.y = 1
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.linear.x = 0
        # else:
        #     FTS_status_msg.twist.linear.x = 1
        # self.FTS_status_pub.publish(FTS_status_msg)
        # #publish Temperature
        # temperature_msg = Float64()
        # temperature_msg.data = np.float64((TxPDO_Temperature).value)
        # self.Temperature_pub.publish(temperature_msg)

        slave.dc_sync(1, 10000000)

    def getSensor(self):
        ### TxPDO ###
        TxPDO_Fx = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 1))
        TxPDO_Fy = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 2))
        TxPDO_Fz = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 3))
        TxPDO_Tx = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 4))
        TxPDO_Ty = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 5))
        TxPDO_Tz = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 6))
        # TxPDO_FTS_Status = int.from_bytes(self.slave.sdo_read(0x6000, 7), byteorder='little', signed=False)
        # TxPDO_Temperature = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 8))
        # publish FT data
        # tf_data_msg = TwistStamped()
        tf_data_msg = WrenchStamped()
        tf_data_msg.header.stamp = rospy.Time.now()
        tf_data_msg.header.frame_id = 'RFT64-6A01'
        tf_data_msg.wrench.force.x = np.float64((TxPDO_Fx).value)
        tf_data_msg.wrench.force.y = np.float64((TxPDO_Fy).value)
        tf_data_msg.wrench.force.z = np.float64((TxPDO_Fz).value)
        tf_data_msg.wrench.torque.x = np.float64((TxPDO_Tx).value)
        tf_data_msg.wrench.torque.y = np.float64((TxPDO_Ty).value)
        tf_data_msg.wrench.torque.z = np.float64((TxPDO_Tz).value)
        
        self.FT_data_pub.publish(tf_data_msg)
        # publish FTS-Status
        # FTS_status_msg = TwistStamped()
        # FTS_status_msg.header = tf_data_msg.header
        # share = 0
        # if (TxPDO_FTS_Status % 2 == 0):
        #     FTS_status_msg.twist.angular.z = 0
        # else:
        #     FTS_status_msg.twist.angular.z = 1
        # share = TxPDO_FTS_Status // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.angular.y = 0
        # else:
        #     FTS_status_msg.twist.angular.y = 1
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.angular.x = 0
        # else:
        #     FTS_status_msg.twist.angular.x = 1
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.linear.z = 0
        # else:
        #     FTS_status_msg.twist.linear.z = 1   
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.linear.y = 0
        # else:
        #     FTS_status_msg.twist.linear.y = 1
        # share =  share // 2
        # if (share % 2 == 0):
        #     FTS_status_msg.twist.linear.x = 0
        # else:
        #     FTS_status_msg.twist.linear.x = 1
        # self.FTS_status_pub.publish(FTS_status_msg)
        #publish Temperature
        # temperature_msg = Float64()
        # temperature_msg.data = np.float64((TxPDO_Temperature).value)
        # self.Temperature_pub.publish(temperature_msg)

    def run(self):
        # time1 = rospy.Time.now().to_sec()
        # self._master = pysoem.Master()
        # self._master.open(self._ifname)

        # if not self._master.config_init() > 0:
        #     self._master.close()
        #     raise InterfaceError('no slave found')
        # for i, slave in enumerate(self._master.slaves):
        #     if not ((slave.man == self.BECKHOFF_VENDOR_ID) and
        #             (slave.id == self._expected_slave_layout[i].product_code)):
        #         self._master.close()
        #         raise InterfaceError('unexpected slave layout')
        #     slave.config_func = self._expected_slave_layout[i].config_func
        #     slave.is_lost = False
        
        # self._master.read_state()
        # self._master.receive_processdata()
        # print(i)
        # self._expected_slave_layout[0].config_func
        self.getSensor()
        # is_lost = False
        # for i, slave in enumerate(self._master.slaves):
        #     print(i)
        #     slave.config_func = self._expected_slave_layout[0].config_func
        #     slave.is_lost = False
        # time2 = rospy.Time.now().to_sec()
        # print(time2-time1)

        # self._master.close()

class InterfaceError(Exception):
    def __init__(self, message):
        super(InterfaceError, self).__init__(message)
        self.message = message

def main():
    interface = RFT64(sys.argv[1])

    if len(sys.argv) > 1:
        # interface._master = pysoem.Master()
        interface._master.open(sys.argv[1])
        try:
            while not rospy.is_shutdown():
                interface.getSensor()
                # model_name_msg = String()
                # model_name_msg.data = interface.Model_name
                # interface.Model_name_pub.publish(model_name_msg)
                interface.rate.sleep()
        except InterfaceError as expt:
            print('RFT64 failed: ' + expt.message)
            sys.exit(1)
    else:
        print('usage: RFT64 ifname')
        sys.exit(1)


if __name__ == '__main__':
    main()
