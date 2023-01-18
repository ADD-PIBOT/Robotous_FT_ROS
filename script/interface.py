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

from geometry_msgs.msg import Wrench, WrenchStamped
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String

from rft64_6a01_ros_interface.srv import set_LPF_rate, set_LPF_rateResponse
from rft64_6a01_ros_interface.srv import set_as_bias, set_as_biasResponse

class RFT64:
    BECKHOFF_VENDOR_ID = 0x000008EE
    TFSENSOR_PRODUCT_CODE = 0x00000003

    def __init__(self, ifname):
        rospy.init_node('RFT64')
        # rospy.init_node('RFT64', anonymous=True)
        self.rate = rospy.Rate(500)

        self._ifname = ifname
        self._pd_thread_stop_event = threading.Event()
        self._ch_thread_stop_event = threading.Event()
        self._actual_wkc = 0
        self._master = pysoem.Master()
        self._master.in_op = False
        self._master.do_check_state = False
        SlaveSet = namedtuple('SlaveSet', 'name product_code config_func')
        self._expected_slave_layout = {0: SlaveSet('RFT64', self.TFSENSOR_PRODUCT_CODE, self.tfsensor_setup)}
                                    #  1: SlaveSet('RFT64', self.TFSENSOR_PRODUCT_CODE, self.tfsensor_setup)}
        self.collapse_time = 0.0

        self._bias = Wrench()

        # ROS publisher
        self.Model_name_pub = rospy.Publisher("RFT64/Model_name", String, queue_size=1)
        self.Serial_number_pub = rospy.Publisher("RFT64/Serial_number", String, queue_size=1)
        self.Firmware_version_pub = rospy.Publisher("RFT64/Firmware_version", String, queue_size=1)

        self.Biased_pub = rospy.Publisher("RFT64/Bias", Int16, queue_size=1)
        self.LPF_setup_pub = rospy.Publisher("RFT64/LPF_setup", Int16, queue_size=1)

        self.Overload_count_pub = rospy.Publisher("RFT64/Overload_count", Wrench, queue_size=1)

        self.FT_data_pub = rospy.Publisher("RFT64/FT_data", WrenchStamped, queue_size=1)
        self.FTS_status_pub = rospy.Publisher("RFT64/Overloaded", WrenchStamped, queue_size=1)
        self.Temperature_pub = rospy.Publisher("RFT64/Temperature", Float64, queue_size=1)

        self.set_LPF_server = rospy.Service("set_FT_LPF", set_LPF_rate, self.set_ft_rate)
        self.set_bias_server = rospy.Service("set_as_bias", set_as_bias, self.set_as_bias)
        self.unset_bias_server = rospy.Service("unset_bias", set_as_bias, self.unset_bias)

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

    def set_ft_rate(self, req):
        rate = req.rate
        self.slave.sdo_write(0x2001, 2, struct.pack('h', rate.data))
        new_rate = int.from_bytes(self.slave.sdo_read(0x2001, 2), byteorder='little', signed=False)
        res = set_LPF_rateResponse()
        res.rate.data = new_rate
        return res

    def set_as_bias(self, req):
        # New way to setting BIAS. But it is not available for RFT80 or RFT64
        # self.slave.sdo_write(0x7000, 1, struct.pack('I', 0x0000000B))
        # self.slave.sdo_write(0x7000, 1, struct.pack('I', 0x00000011))

        # Pseudo Biasing
        self.set_cur_as_bias()

        res = set_as_biasResponse()
        return res

    def set_cur_as_bias(self):
        self._bias.force.x = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 1)).value
        self._bias.force.y = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 2)).value
        self._bias.force.z = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 3)).value
        self._bias.torque.x = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 4)).value
        self._bias.torque.y = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 5)).value
        self._bias.torque.z = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 6)).value
        self.Biased = True
    
    def unset_bias(self, req):
        self._bias.force.x = 0.0
        self._bias.force.y = 0.0
        self._bias.force.z = 0.0
        self._bias.torque.x = 0.0
        self._bias.torque.y = 0.0
        self._bias.torque.z = 0.0
        self.Biased = False

        res = set_as_biasResponse()
        return res

    def pub_sensor_info(self):
        # publish model name
        model_name_msg = String()
        model_name_msg.data = self.Model_name
        self.Model_name_pub.publish(model_name_msg)

        #publish serial number
        serial_number_msg = String()
        serial_number_msg.data = self.Serial_number
        self.Serial_number_pub.publish(serial_number_msg)

        # publish firmware version
        firmware_version_msg = String()
        firmware_version_msg.data = self.Firmware_version
        self.Firmware_version_pub.publish(firmware_version_msg)

        # publish Biased
        Biased_msg = Int16()
        Biased_msg.data = self.Biased
        self.Biased_pub.publish(Biased_msg)

        # publish LPF_Setup
        LPF_setup_msg = Int16()
        LPF_setup_msg.data = self.LPF_setup
        self.LPF_setup_pub.publish(LPF_setup_msg)

        # publish Overload count 
        overload_count_msg = Wrench()
        overload_count_msg.force.x = self.overload_count_Fx
        overload_count_msg.force.y = self.overload_count_Fy
        overload_count_msg.force.z = self.overload_count_Fz
        overload_count_msg.torque.x = self.overload_count_Tx
        overload_count_msg.torque.y = self.overload_count_Ty
        overload_count_msg.torque.z = self.overload_count_Tz
        self.Overload_count_pub.publish(overload_count_msg)

    def tfsensor_setup(self, slave_pos):
        print("Initialize sensor...")
        slave = self._master.slaves[slave_pos]
        self.slave = slave

        ### Sensor Information ###
        self.Model_name = slave.sdo_read(0x2000, 1).decode('utf-8')
        self.Serial_number = slave.sdo_read(0x2000, 2).decode('utf-8')
        self.Firmware_version = slave.sdo_read(0x2000, 3).decode('utf-8')

        ### Sensor Setup ###
        self.Biased = int.from_bytes(slave.sdo_read(0x2001, 1), byteorder='little', signed=False)
        self.LPF_setup = int.from_bytes(slave.sdo_read(0x2001, 2), byteorder='little', signed=False)

        ### Overload Counter ###
        self.overload_count_Fx = int.from_bytes(slave.sdo_read(0x2002, 1), byteorder='little', signed=False)
        self.overload_count_Fy = int.from_bytes(slave.sdo_read(0x2002, 2), byteorder='little', signed=False)
        self.overload_count_Fz = int.from_bytes(slave.sdo_read(0x2002, 3), byteorder='little', signed=False)
        self.overload_count_Tx = int.from_bytes(slave.sdo_read(0x2002, 4), byteorder='little', signed=False)
        self.overload_count_Ty = int.from_bytes(slave.sdo_read(0x2002, 5), byteorder='little', signed=False)
        self.overload_count_Tz = int.from_bytes(slave.sdo_read(0x2002, 6), byteorder='little', signed=False)

        slave.dc_sync(1, 10000000)

    def getSensor(self):
        ### TxPDO ###

        # Force-torque
        TxPDO_Fx = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 1))
        TxPDO_Fy = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 2))
        TxPDO_Fz = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 3))
        TxPDO_Tx = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 4))
        TxPDO_Ty = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 5))
        TxPDO_Tz = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 6))
        TxPDO_FTS_Status = int.from_bytes(self.slave.sdo_read(0x6000, 7), byteorder='little', signed=False)
        # Temperature
        TxPDO_Temperature = ctypes.c_float.from_buffer_copy(self.slave.sdo_read(0x6000, 8))

        # Process FT data
        tf_data_msg = WrenchStamped()
        tf_data_msg.header.stamp = rospy.Time.now()
        tf_data_msg.header.frame_id = 'RFT64-6A01'
        tf_data_msg.wrench.force.x = TxPDO_Fx.value - self._bias.force.x
        tf_data_msg.wrench.force.y = TxPDO_Fy.value - self._bias.force.y
        tf_data_msg.wrench.force.z = TxPDO_Fz.value - self._bias.force.z
        tf_data_msg.wrench.torque.x = TxPDO_Tx.value - self._bias.torque.x
        tf_data_msg.wrench.torque.y = TxPDO_Ty.value - self._bias.torque.y
        tf_data_msg.wrench.torque.z = TxPDO_Tz.value - self._bias.torque.z
        
        # Process FTS-Status
        FTS_status_msg = WrenchStamped()
        FTS_status_msg.header = tf_data_msg.header

        share = 0
        if (TxPDO_FTS_Status % 2 == 1):
            FTS_status_msg.wrench.torque.z = 1
        share = TxPDO_FTS_Status // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.x = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.z = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.x = 1

        # Process Temperature
        temperature_msg = Float64()
        temperature_msg.data = TxPDO_Temperature.value

        # Publish messages
        self.FT_data_pub.publish(tf_data_msg)
        self.FTS_status_pub.publish(FTS_status_msg)
        self.Temperature_pub.publish(temperature_msg)

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
                interface.pub_sensor_info()
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
