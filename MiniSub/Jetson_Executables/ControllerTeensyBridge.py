import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt64, Int32, Float64, Int16, Bool
import math

class Joystick:

	def __init__(self):
		self.TSData = UInt64()
		self.button_input = [0 for i in range(11)]
		self.axes_input = [0 for i in range(8)]
		rospy.Subscriber('/joy', Joy, self.Joystick_CallBack)
		self.TSPub = rospy.Publisher('/ThrusterStates', UInt64, queue_size = 10)		

	def Joystick_CallBack(self,joy):
		#button array[A B X Y LB RB BACK START POWER LB RB]
		for i in range(len(self.button_input)):
			self.button_input[i] = joy.buttons[i]	
		#axes array [LS_X LS...
		for i in range(len(self.axes_input)):
			self.axes_input[i] = joy.axes[i]	
		self.execute()
		
	def execute(self):
		self.TSData.data = 0x7D7D7D7D7D7D7D7D # all thrusters set to 125 (OFF)
		if self.button_input[5]: #if Lb is pressed, CCW barrel roll
			self.TSData.data |= 0x00FFFF0000000000
			self.TSData.data &= 0xFFFFFFFFFF0000FF

		if self.button_input[4]:#if rb is pressed, CW barrel roll
			self.TSData.data |= 0x000000000FFFFF00
			self.TSData.data &= 0xFF0000FFFFFFFFFF
		
		if self.button_input[0]: #if A is pressed, go forward
			self.TSData.data |= (0xFF <<0*8)
			self.TSData.data &= ~(0xFF <<3*8)
			self.TSData.data &= ~(0xFF <<4*8)
			self.TSData.data |= (0xFF <<7*8)
		
		if self.button_input[1]: #if B is pressed, all blue
			self.TSData.data = 0		

		if self.axes_input[3]: #left joystick horizontal axi
			axi_hor_data = (int)(((self.axes_input[3] + 1) / 2) * 255)
			self.TSData.data = ((255-axi_hor_data) << 0*8) + (axi_hor_data << 3*8) + ((255-axi_hor_data) << 4*8)+ (axi_hor_data << 7*8)
			self.TSData.dat  a &= 0xFF0000FFFF0000FF
			self.TSData.data |= 0x007D7D00007D7D00
		
		if self.axes_input[4]: #left joystick vertical axi
			axi_ver_data = (int)(((self.axes_input[4] + 1) / 2) * 255) 
			self.TSData.data = (axi_ver_data << 0*8) + (axi_ver_data << 3*8) + (axi_ver_data << 4*8)+ (axi_ver_data << 7*8)
			self.TSData.data &= 0xFF0000FFFF0000FF
			self.TSData.data |= 0x007D7D00007D7D00
		
		#publish data
		self.TSPub.publish(self.TSData)
	

def main():
	rospy.init_node('joystickController')
	rate = rospy.Rate(20)

	joystick = Joystick();

	print("Joy-Teensy Bridge Running...")
	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == "__main__":
	main()
