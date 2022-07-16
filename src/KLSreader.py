import time
import rospy
from pprint import pprint
from serial import Serial, PARITY_NONE, STOPBITS_ONE
from protocol.controllerdata import *
from protocol.controllercommand import *
from sys import platform

from std_msgs.msg import Int32, Float32 ,Float64
from control_msgs.msg import Velocity

from morai_msgs.msg import CtrlCmd

# set RS232 port name
rospy.set_param('PORT', '/dev/ttyUSB0')

class ControllerConnector(object):
    def __init__(self):
        self.serialport = rospy.get_param('PORT')

    def startSerial(self):
        self.connection = Serial(self.serialport, 19200, timeout=5,parity=PARITY_NONE, stopbits=STOPBITS_ONE)

    def getBytes(self, *commands):
        ser = self.connection
        packets = []
        for command in commands:
            ser.write(command)
            packet = ser.read(19)
            packets.append(packet)
        return packets

class KLSReader(object):
    def __init__(self, serialport):
        self.connector = ControllerConnector(serialport)
        self.connector.startSerial()
        self.command = ControllerCommand()

    def getData(self):
        packet_a, packet_b = self.connector.getBytes(self.command.a, self.command.b)
        data = ControllerData(packet_a, packet_b)
        return data.__dict__

if __name__ == "__main__":
    rospy.init_node('/vehicle_status') # set node

    # PUBLISH
    pub_speed_car = rospy.Publisher('/vehicle_velocity', Velocity, queue_size=10)

    # SUBSCRIBE
    #rospy.Subscriber('/ctrl_cmd',CtrlCmd,cmd_callback)

    controller = KLSReader()
    print("Connected to motor controller")

    try:
        while not rospy.is_shutdown():
            data = controller.getData()
            pprint(data)

            pub_speed_car.publish(data.get('rpm'))
            time.sleep(1)

    except KeyboardInterrupt:
        print("Terminating connection")
