#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

from pty import slave_open
from typing import overload
import rospy

import sys
import struct
import threading
import ctypes
from collections import namedtuple
import pysoem

from geometry_msgs.msg import Wrench, WrenchStamped
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import String

from rft64_6a01_ros_interface.srv import set_LPF_rate, set_LPF_rateResponse
from rft64_6a01_ros_interface.srv import set_as_bias, set_as_biasResponse

class ROBOTOUS_FT:
    BECKHOFF_VENDOR_ID = 0x00000002
    CU1128_PRODUCT_CODE = 0x04685432

    ROBOTOUS_VENDOR_ID = 0x000008EE
    RFT64_6A01_PRODUCT_CODE = 0x00000003

    def __init__(self, ifname):
        self._ifname = ifname
        self._actual_wkc = 0
        self._info_pub_div = 1000
        self._counter = 0

        self.Model_name = []
        self.Serial_number = []
        self.Firmware_version = []
        self.Biased = []
        self.LPF_setup = []
        self.overload_count_Fx = []
        self.overload_count_Fy = []
        self.overload_count_Fz = []
        self.overload_count_Tx = []
        self.overload_count_Ty = []
        self.overload_count_Tz = []

        self._master = pysoem.Master()
        self._master.in_op = False
        self._master.do_check_state = False

        SlaveSet = namedtuple('SlaveSet', 'name product_code config_func')
        self._expected_slave_layout = {0: SlaveSet('CU1128', self.CU1128_PRODUCT_CODE, self.cu1128_setup)
                                        ,1: SlaveSet('RFT80', self.RFT64_6A01_PRODUCT_CODE, self.tfsensor_setup)
                                        ,2: SlaveSet('RFT64', self.RFT64_6A01_PRODUCT_CODE, self.tfsensor_setup)
                                        ,3: SlaveSet('CU1128-C', self.CU1128_PRODUCT_CODE, self.cu1128_setup)
                                        ,4: SlaveSet('CU1128-D', self.CU1128_PRODUCT_CODE, self.cu1128_setup)}
        
        self.slave_list_lengh = len(self._expected_slave_layout)
        print("Expected slave length: ", self.slave_list_lengh)
        # self.collapse_time = 0.0

        self._master.open(self._ifname)
        if not self._master.config_init() > 0:
            self._master.close()
            raise InterfaceError('no slave found')
        
        for i, slave in enumerate(self._master.slaves):
            print("slave add ...")
            if not (((slave.man == self.BECKHOFF_VENDOR_ID) or (slave.man == self.ROBOTOUS_VENDOR_ID)) and
                    (slave.id == self._expected_slave_layout[i].product_code)):
                self._master.close()
                raise InterfaceError('unexpected slave layout')
            slave.config_func = self._expected_slave_layout[i].config_func
            slave.is_lost = False

        print("Sensor initalized with PDO map: {}", self._master.config_map())

        # ROS
        rospy.init_node('robotous_ft')
        # rospy.init_node('RFT64', anonymous=True)
        self.rate = rospy.Rate(1000)
        
        # ROS publisher
        self.FT_set_LPF_server = rospy.Service("FT2/set_ft_lpf", set_LPF_rate, self.FT_set_ft_rate)
        self.FT_set_bias_server = rospy.Service("FT2/set_as_bias", set_as_bias, self.FT_set_as_bias)
        self.FT_unset_bias_server = rospy.Service("FT2/unset_bias", set_as_bias, self.FT_unset_bias)

        ### FT sensor 1
        self.FT1_Model_name_pub = rospy.Publisher("FT1/Model_name", String, queue_size=1)
        self.FT1_Serial_number_pub = rospy.Publisher("FT1/Serial_number", String, queue_size=1)
        self.FT1_Firmware_version_pub = rospy.Publisher("FT1/Firmware_version", String, queue_size=1)

        self.FT1_Biased_pub = rospy.Publisher("FT1/Bias", Int16, queue_size=1)
        self.FT1_LPF_setup_pub = rospy.Publisher("FT1/LPF_setup", Int16, queue_size=1)

        self.FT1_Overload_count_pub = rospy.Publisher("FT1/Overload_count", Wrench, queue_size=1)

        self.FT1_FT_data_pub = rospy.Publisher("FT1/data", WrenchStamped, queue_size=1)
        self.FT1_FTS_status_pub = rospy.Publisher("FT1/Overloaded", WrenchStamped, queue_size=1)
        self.FT1_Temperature_pub = rospy.Publisher("FT1/Temperature", Float64, queue_size=1)

        self.FT1_bias = Wrench()

        ### FT sensor 2
        self.FT2_Model_name_pub = rospy.Publisher("FT2/Model_name", String, queue_size=1)
        self.FT2_Serial_number_pub = rospy.Publisher("FT2/Serial_number", String, queue_size=1)
        self.FT2_Firmware_version_pub = rospy.Publisher("FT2/Firmware_version", String, queue_size=1)

        self.FT2_Biased_pub = rospy.Publisher("FT2/Bias", Int16, queue_size=1)
        self.FT2_LPF_setup_pub = rospy.Publisher("FT2/LPF_setup", Int16, queue_size=1)

        self.FT2_Overload_count_pub = rospy.Publisher("FT2/Overload_count", Wrench, queue_size=1)

        self.FT2_FT_data_pub = rospy.Publisher("FT2/data", WrenchStamped, queue_size=1)
        self.FT2_FTS_status_pub = rospy.Publisher("FT2/Overloaded", WrenchStamped, queue_size=1)
        self.FT2_Temperature_pub = rospy.Publisher("FT2/Temperature", Float64, queue_size=1)

        self.FT2_bias = Wrench()

        print('RFT64 interface node started..!')

    def FT_set_ft_rate(self, req):
        rate = req.rate
        self._master.slaves[req.sensor_idx].sdo_write(0x2001, 2, struct.pack('h', rate))
        new_rate = int.from_bytes(self._master.slaves[req.sensor_idx].sdo_read(0x2001, 2), byteorder='little', signed=False)
        res = set_LPF_rateResponse()
        res.rate = new_rate
        return res

    def FT_set_as_bias(self, req):
        # New way to setting BIAS. But it is not available for RFT80_6A01 or RFT64_6A02
        # self.slave.sdo_write(0x7000, 1, struct.pack('I', 0x0000000B))
        # self.slave.sdo_write(0x7000, 1, struct.pack('I', 0x00000011))

        # Pseudo Biasing
        self.FT_set_cur_as_bias(req)

        res = set_as_biasResponse()
        return res

    def FT_set_cur_as_bias(self, req):
        if(req.sensor_idx == 1):
            self.FT1_bias.force.x = ctypes.c_float.from_buffer_copy(self._master.slaves[1].sdo_read(0x6000, 1)).value
            self.FT1_bias.force.y = ctypes.c_float.from_buffer_copy(self._master.slaves[1].sdo_read(0x6000, 2)).value
            self.FT1_bias.force.z = ctypes.c_float.from_buffer_copy(self._master.slaves[1].sdo_read(0x6000, 3)).value
            self.FT1_bias.torque.x = ctypes.c_float.from_buffer_copy(self._master.slaves[1].sdo_read(0x6000, 4)).value
            self.FT1_bias.torque.y = ctypes.c_float.from_buffer_copy(self._master.slaves[1].sdo_read(0x6000, 5)).value
            self.FT1_bias.torque.z = ctypes.c_float.from_buffer_copy(self._master.slaves[1].sdo_read(0x6000, 6)).value
            self.FT1_Biased = True
        elif(req.sensor_idx == 2):
            self.FT2_bias.force.x = ctypes.c_float.from_buffer_copy(self._master.slaves[2].sdo_read(0x6000, 1)).value
            self.FT2_bias.force.y = ctypes.c_float.from_buffer_copy(self._master.slaves[2].sdo_read(0x6000, 2)).value
            self.FT2_bias.force.z = ctypes.c_float.from_buffer_copy(self._master.slaves[2].sdo_read(0x6000, 3)).value
            self.FT2_bias.torque.x = ctypes.c_float.from_buffer_copy(self._master.slaves[2].sdo_read(0x6000, 4)).value
            self.FT2_bias.torque.y = ctypes.c_float.from_buffer_copy(self._master.slaves[2].sdo_read(0x6000, 5)).value
            self.FT2_bias.torque.z = ctypes.c_float.from_buffer_copy(self._master.slaves[2].sdo_read(0x6000, 6)).value
            self.FT2_Biased = True
    
    def FT_unset_bias(self, req):
        if(req.sensor_idx == 1):
            self.FT1_bias.force.x = 0.0
            self.FT1_bias.force.y = 0.0
            self.FT1_bias.force.z = 0.0
            self.FT1_bias.torque.x = 0.0
            self.FT1_bias.torque.y = 0.0
            self.FT1_bias.torque.z = 0.0
            self.FT1_Biased = False
        elif(req.sensor_idx == 2):
            self.FT2_bias.force.x = 0.0
            self.FT2_bias.force.y = 0.0
            self.FT2_bias.force.z = 0.0
            self.FT2_bias.torque.x = 0.0
            self.FT2_bias.torque.y = 0.0
            self.FT2_bias.torque.z = 0.0
            self.FT2_Biased = False
        res = set_as_biasResponse()
        return res

    def pub_sensor_info(self):
        # publish model name
        model_name_msg1 = String()
        model_name_msg1.data = self.Model_name[0]
        self.FT1_Model_name_pub.publish(model_name_msg1)

        model_name_msg2 = String()
        model_name_msg2.data = self.Model_name[1]
        self.FT2_Model_name_pub.publish(model_name_msg2)

        #publish serial number
        serial_number_msg1 = String()
        serial_number_msg1.data = self.Serial_number[0]
        self.FT1_Serial_number_pub.publish(serial_number_msg1)

        serial_number_msg2 = String()
        serial_number_msg2.data = self.Serial_number[1]
        self.FT1_Serial_number_pub.publish(serial_number_msg2)

        # publish firmware version
        firmware_version_msg1 = String()
        firmware_version_msg1.data = self.Firmware_version[0]
        self.FT1_Firmware_version_pub.publish(firmware_version_msg1)

        firmware_version_msg2 = String()
        firmware_version_msg2.data = self.Firmware_version[1]
        self.FT1_Firmware_version_pub.publish(firmware_version_msg2)

        # publish Biased
        Biased_msg1 = Int16()
        Biased_msg1.data = self.Biased[0]
        self.FT1_Biased_pub.publish(Biased_msg1)

        Biased_msg2 = Int16()
        Biased_msg2.data = self.Biased[1]
        self.FT1_Biased_pub.publish(Biased_msg2)

        # publish LPF_Setup
        LPF_setup_msg1 = Int16()
        LPF_setup_msg1.data = self.LPF_setup[0]
        self.FT1_LPF_setup_pub.publish(LPF_setup_msg1)

        LPF_setup_msg2 = Int16()
        LPF_setup_msg2.data = self.LPF_setup[1]
        self.FT1_LPF_setup_pub.publish(LPF_setup_msg2)

        # publish Overload count
        overload_count_msg1 = Wrench()
        overload_count_msg1.force.x = self.overload_count_Fx[0]
        overload_count_msg1.force.y = self.overload_count_Fy[0]
        overload_count_msg1.force.z = self.overload_count_Fz[0]
        overload_count_msg1.torque.x = self.overload_count_Tx[0]
        overload_count_msg1.torque.y = self.overload_count_Ty[0]
        overload_count_msg1.torque.z = self.overload_count_Tz[0]
        self.FT1_Overload_count_pub.publish(overload_count_msg1)

        overload_count_msg2 = Wrench()
        overload_count_msg2.force.x = self.overload_count_Fx[1]
        overload_count_msg2.force.y = self.overload_count_Fy[1]
        overload_count_msg2.force.z = self.overload_count_Fz[1]
        overload_count_msg2.torque.x = self.overload_count_Tx[1]
        overload_count_msg2.torque.y = self.overload_count_Ty[1]
        overload_count_msg2.torque.z = self.overload_count_Tz[1]
        self.FT2_Overload_count_pub.publish(overload_count_msg2)

    def cu1128_setup(self, slave_pos):
        slave = self._master.slaves[slave_pos]

    def tfsensor_setup(self, slave_pos):
        print("Initialize sensor...")
        # slave = self._master.slaves[slave_pos]
        # self.slave = slave

        ### Sensor Information ###
        name_list = self._master.slaves[slave_pos].sdo_read(0x2000, 1).decode('utf-8')
        self.Model_name.append(name_list.rstrip('\0')) 
        self.Serial_number.append(self._master.slaves[slave_pos].sdo_read(0x2000, 2).decode('utf-8'))
        self.Firmware_version.append(self._master.slaves[slave_pos].sdo_read(0x2000, 3).decode('utf-8'))

        ### Sensor Setup ###
        self.Biased.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2001, 1), byteorder='little', signed=False))
        self.LPF_setup.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2001, 2), byteorder='little', signed=False))

        ### Overload Counter ###
        self.overload_count_Fx.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2002, 1), byteorder='little', signed=False))
        self.overload_count_Fy.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2002, 2), byteorder='little', signed=False))
        self.overload_count_Fz.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2002, 3), byteorder='little', signed=False))
        self.overload_count_Tx.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2002, 4), byteorder='little', signed=False))
        self.overload_count_Ty.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2002, 5), byteorder='little', signed=False))
        self.overload_count_Tz.append(int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x2002, 6), byteorder='little', signed=False))
        print("sensor ", slave_pos, " initialized!")
        

    def getSensor(self, slave_pos):
        ### TxSDO ###
        # Force-torque
        TxPDO_Fx = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 1))
        TxPDO_Fy = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 2))
        TxPDO_Fz = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 3))
        TxPDO_Tx = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 4))
        TxPDO_Ty = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 5))
        TxPDO_Tz = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 6))
        TxPDO_FTS_Status = int.from_bytes(self._master.slaves[slave_pos].sdo_read(0x6000, 7), byteorder='little', signed=False)
        # Temperature
        TxPDO_Temperature = ctypes.c_float.from_buffer_copy(self._master.slaves[slave_pos].sdo_read(0x6000, 8))

        # Process FT data
        tf_data_msg = WrenchStamped()
        tf_data_msg.header.stamp = rospy.Time.now()
        tf_data_msg.header.frame_id = self.Model_name[slave_pos - 1]
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
        pdos1 = self._master.slaves[1].input
        pdos2 = self._master.slaves[2].input
        Fx1, Fy1, Fz1, Tx1, Ty1, Tz1, FT_Status1, Temp1 = struct.unpack('ffffffIf', pdos1)
        Fx2, Fy2, Fz2, Tx2, Ty2, Tz2, FT_Status2, Temp2 = struct.unpack('ffffffIf', pdos2)

        # Process FT data
        ft1_data_msg = WrenchStamped()
        ft1_data_msg.header.stamp = rospy.Time.now()
        ft1_data_msg.header.frame_id = self.Model_name[0]
        ft1_data_msg.wrench.force.x = Fx1 - self.FT1_bias.force.x
        ft1_data_msg.wrench.force.y = Fy1 - self.FT1_bias.force.y
        ft1_data_msg.wrench.force.z = Fz1 - self.FT1_bias.force.z
        ft1_data_msg.wrench.torque.x = Tx1 - self.FT1_bias.torque.x
        ft1_data_msg.wrench.torque.y = Ty1 - self.FT1_bias.torque.y
        ft1_data_msg.wrench.torque.z = Tz1 - self.FT1_bias.torque.z

        ft2_data_msg = WrenchStamped()
        ft2_data_msg.header.stamp = rospy.Time.now()
        ft2_data_msg.header.frame_id = self.Model_name[1]
        ft2_data_msg.wrench.force.x = Fx2 - self.FT2_bias.force.x
        ft2_data_msg.wrench.force.y = Fy2 - self.FT2_bias.force.y
        ft2_data_msg.wrench.force.z = Fz2 - self.FT2_bias.force.z
        ft2_data_msg.wrench.torque.x = Tx2 - self.FT2_bias.torque.x
        ft2_data_msg.wrench.torque.y = Ty2 - self.FT2_bias.torque.y
        ft2_data_msg.wrench.torque.z = Tz2 - self.FT2_bias.torque.z
        
        # Process FTS-Status
        FTS1_status_msg = WrenchStamped()
        FTS1_status_msg.header = ft1_data_msg.header

        share = 0
        if (FT_Status1 % 2 == 1):
            FTS1_status_msg.wrench.torque.z = 1
        share = FT_Status1 // 2
        if (share % 2 == 1):
            FTS1_status_msg.wrench.torque.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS1_status_msg.wrench.torque.x = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS1_status_msg.wrench.force.z = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS1_status_msg.wrench.force.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS1_status_msg.wrench.force.x = 1

        FTS2_status_msg = WrenchStamped()
        FTS2_status_msg.header = ft1_data_msg.header

        share = 0
        if (FT_Status2 % 2 == 1):
            FTS2_status_msg.wrench.torque.z = 1
        share = FT_Status2 // 2
        if (share % 2 == 1):
            FTS2_status_msg.wrench.torque.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS2_status_msg.wrench.torque.x = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS2_status_msg.wrench.force.z = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS2_status_msg.wrench.force.y = 1
        share =  share // 2
        if (share % 2 == 1):
            FTS2_status_msg.wrench.force.x = 1

        # Process Temperature
        temperature1_msg = Float64()
        temperature1_msg.data = Temp1

        temperature2_msg = Float64()
        temperature2_msg.data = Temp2

        self.FT1_FT_data_pub.publish(ft1_data_msg)
        self.FT1_FTS_status_pub.publish(FTS1_status_msg)
        self.FT1_Temperature_pub.publish(temperature1_msg)

        self.FT2_FT_data_pub.publish(ft2_data_msg)
        self.FT2_FTS_status_pub.publish(FTS2_status_msg)
        self.FT2_Temperature_pub.publish(temperature2_msg)

    def run(self):
        self._master.state = pysoem.OP_STATE
        self._master.write_state()

        try:
            while not rospy.is_shutdown():
                # interface.getSensor()
                self.getSensorPDO()
                if(self._counter > self._info_pub_div):
                    self.pub_sensor_info()
                    self._counter = 0

                self._counter = self._counter + 1
                self.rate.sleep()
        except InterfaceError as expt:
            print('Robotous ROS Interface failed: ' + expt.message)
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
        print('usage: dual_rft ifname')
        sys.exit(1)


if __name__ == '__main__':
    main()
