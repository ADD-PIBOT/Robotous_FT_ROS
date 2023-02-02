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

class ROBOTOUS_FT:
    BECKHOFF_VENDOR_ID = 0x000008EE
    TFSENSOR_PRODUCT_CODE = 0x00000003

    CU1128_VENDOR_ID = 0x00000002
    CU1128_PRODUCT_CODE = 0x04685432

    def __init__(self, ifname):
        self._ifname = ifname
        self._pd_thread_stop_event = threading.Event()
        self._ch_thread_stop_event = threading.Event()
        self._actual_wkc = 0

        self._master = pysoem.Master()
        self._master.in_op = False
        self._master.do_check_state = False

        SlaveSet = namedtuple('SlaveSet', 'name product_code config_func')
        self._expected_slave_layout = {0: SlaveSet('CU1128', self.CU1128_PRODUCT_CODE, self.cu1128_setup)
                                        ,1: SlaveSet('RFT64', self.TFSENSOR_PRODUCT_CODE, self.tfsensor_setup)
                                        ,2: SlaveSet('CU1128-C', self.CU1128_PRODUCT_CODE, self.cu1128_setup)
                                        ,3: SlaveSet('CU1128-D', self.CU1128_PRODUCT_CODE, self.cu1128_setup)}
        # self.collapse_time = 0.0

        self._master.open(self._ifname)
        if not self._master.config_init() > 0:
            self._master.close()
            raise InterfaceError('no slave found')
        
        for i, slave in enumerate(self._master.slaves):
            print("slave add ...")
            if not (((slave.man == self.BECKHOFF_VENDOR_ID) or (slave.man == self.CU1128_VENDOR_ID)) and
                    (slave.id == self._expected_slave_layout[i].product_code)):
                self._master.close()
                raise InterfaceError('unexpected slave layout')
            slave.config_func = self._expected_slave_layout[i].config_func
            slave.is_lost = False

        print("Sensor initalized with PDO map: {}", self._master.config_map())


        self._master.config_map()

        # ROS
        rospy.init_node('robotous_ft')
        # rospy.init_node('RFT64', anonymous=True)
        self.rate = rospy.Rate(1000)
        
        # ROS publisher
        self.Model_name_pub = rospy.Publisher("Model_name", String, queue_size=1)
        self.Serial_number_pub = rospy.Publisher("Serial_number", String, queue_size=1)
        self.Firmware_version_pub = rospy.Publisher("Firmware_version", String, queue_size=1)

        self.Biased_pub = rospy.Publisher("Bias", Int16, queue_size=1)
        self.LPF_setup_pub = rospy.Publisher("LPF_setup", Int16, queue_size=1)

        self.Overload_count_pub = rospy.Publisher("Overload_count", Wrench, queue_size=1)

        self.FT_data_pub = rospy.Publisher("FT_data", WrenchStamped, queue_size=1)
        self.FTS_status_pub = rospy.Publisher("Overloaded", WrenchStamped, queue_size=1)
        self.Temperature_pub = rospy.Publisher("Temperature", Float64, queue_size=1)

        self.set_LPF_server = rospy.Service("set_FT_LPF", set_LPF_rate, self.set_ft_rate)
        self.set_bias_server = rospy.Service("set_as_bias", set_as_bias, self.set_as_bias)
        self.unset_bias_server = rospy.Service("unset_bias", set_as_bias, self.unset_bias)

        self._bias = Wrench()

        print('RFT64 interface node started..!')

    def set_ft_rate(self, req):
        rate = req.rate
        self.slave.sdo_write(0x2001, 2, struct.pack('h', rate.data))
        new_rate = int.from_bytes(self.slave.sdo_read(0x2001, 2), byteorder='little', signed=False)
        res = set_LPF_rateResponse()
        res.rate.data = new_rate
        return res

    def set_as_bias(self, req):
        # New way to setting BIAS. But it is not available for RFT80_6A01 or RFT64_6A02
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

    def cu1128_setup(self, slave_pos):
        slave = self._master.slaves[slave_pos]

    def tfsensor_setup(self, slave_pos):
        print("Initialize sensor...")
        slave = self._master.slaves[slave_pos]
        self.slave = slave

        ### Sensor Information ###
        name_list = slave.sdo_read(0x2000, 1).decode('utf-8')
        self.Model_name = name_list.rstrip('\0')
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
        tf_data_msg.header.frame_id = self.Model_name
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

    def getSensorPDO(self):
        self._master.send_processdata()
        self._master.receive_processdata(2000)
        pdos = self._master.slaves[1].input
        Fx, Fy, Fz, Tx, Ty, Tz, FT_Status, Temp = struct.unpack('ffffffIf', pdos)

        # Process FT data
        tf_data_msg = WrenchStamped()
        tf_data_msg.header.stamp = rospy.Time.now()
        tf_data_msg.header.frame_id = self.Model_name
        tf_data_msg.wrench.force.x = Fx - self._bias.force.x
        tf_data_msg.wrench.force.y = Fy - self._bias.force.y
        tf_data_msg.wrench.force.z = Fz - self._bias.force.z
        tf_data_msg.wrench.torque.x = Tx - self._bias.torque.x
        tf_data_msg.wrench.torque.y = Ty - self._bias.torque.y
        tf_data_msg.wrench.torque.z = Tz - self._bias.torque.z
        
        # Process FTS-Status
        FTS_status_msg = WrenchStamped()
        FTS_status_msg.header = tf_data_msg.header

        share = 0
        if (FT_Status % 2 == 1):
            FTS_status_msg.wrench.torque.z = 1
        share = FT_Status // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.torque.x = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.force.z = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.force.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS_status_msg.wrench.force.x = 1

        # Process Temperature
        temperature_msg = Float64()
        temperature_msg.data = Temp

        self.FT_data_pub.publish(tf_data_msg)
        self.FTS_status_pub.publish(FTS_status_msg)
        self.Temperature_pub.publish(temperature_msg)

    def run(self):
        self._master.state = pysoem.OP_STATE
        self._master.write_state()

        try:
            while not rospy.is_shutdown():
                # interface.getSensor()
                # interface.run()
                self.getSensorPDO()
                self.pub_sensor_info()
                self.rate.sleep()
        except InterfaceError as expt:
            print('RFT64 failed: ' + expt.message)
            sys.exit(1)

class InterfaceError(Exception):
    def __init__(self, message):
        super(InterfaceError, self).__init__(message)
        self.message = message

def main():
    if len(sys.argv) > 1:
        interface = ROBOTOUS_FT(sys.argv[1])
        interface.run()

    else:
        print('usage: RFT64 ifname')
        sys.exit(1)


if __name__ == '__main__':
    main()
