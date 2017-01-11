from mixermagics import *
import lcm
import time
import math

import sys
import os
sys.path.insert(0, os.environ['NJORD_EMBEDDED'] + '/software/build/python/njord')
from servogroup_set_t import *
from force_torque_t import *

SERVO_OFFSET = 3000
SERVO_SWING = 1000

T0 = time.time()

param = MechParam()        
conf = Config11()
cMt = thrMixerMatrix(param, conf)
tMc = np.linalg.inv(cMt)
#desired forces and torques about the camera frame
#                forces    torques
# Start loopisms
foto2 = np.array((0,0,0,0,0,0))
foto3 = np.array((0,0,0,0,0,0))
# * 1000 + 3000
# Cap at [2000, 4000] saturation

def clamp(val, mini=SERVO_OFFSET-SERVO_SWING, maxi=SERVO_OFFSET+SERVO_SWING):
	val = max(min(val, maxi), mini)
	return val

def normalize(val, mini=SERVO_OFFSET-SERVO_SWING, maxi=SERVO_OFFSET+SERVO_SWING):
	n = val	
	if (n > maxi):
		n = maxi
	if n < mini:
		n = mini
	return (n - float(SERVO_OFFSET)) / float(SERVO_SWING)

def depth_handler(channel, data):
	global foto2
	msg = force_torque_t.decode(data)
	foto2 = np.array((msg.force[0],msg.force[1],msg.force[2],0,0,0))

def forces_handler(channel, data):
	global foto3
	msg = force_torque_t.decode(data)
	foto3 = np.array((msg.force[0],msg.force[1],msg.force[2],0,0,0))

def attitude_handler(channel, data):
	msg = force_torque_t.decode(data)

	#forward_force = SERVO_OFFSET + msg.force[0] * SERVO_SWING
	#left_force = SERVO_OFFSET + msg.force[1]  * SERVO_SWING
	#up_force = SERVO_OFFSET + msg.force[2]  * SERVO_SWING
	
	foto1 = np.array((0,0,0,msg.torque[0],msg.torque[1],msg.torque[2]))
	fotot = foto1 + foto2 + foto3
	fth = np.dot(tMc,fotot)*SERVO_SWING + SERVO_OFFSET*np.ones(6)
	# print 'fth', fth
	
	# clamp torques
	for i in range(6):
		clamp(fth[i])
	msgout = servogroup_set_t()
	msgout.utime = int(round((time.time()-T0)*1000000))
	msgout.servo_setpoints = (fth[0],fth[1],fth[2],fth[3],fth[4],fth[5]) 
	msgout.servo_setpoints_norm = (normalize(fth[0]), normalize(fth[1]), normalize(fth[2]), normalize(fth[3]), normalize(fth[4]), normalize(fth[5])) 
	lc.publish("NJORD_V2V_VEH1_SET_SERVOGROUP", msgout.encode())
	# print msgout.servo_setpoints

lc = lcm.LCM()
subscription = lc.subscribe("NJORD_V2V_VEH1_ATTCON_OUTPUTSP", attitude_handler)
subscription = lc.subscribe("NJORD_V2V_VEH1_DEPCON_OUTPUTSP", depth_handler)
subscription = lc.subscribe("NJORD_V2V_VEH1_FORCON_OUTPUTSP", forces_handler)

try:
	while True:
		lc.handle()
except KeyboardInterrupt:
	pass

# msg = servogroup_set_t()

# omega = 2.0*3.14159265358979323*0.5

# T0 = time.time()
# while (True):
#   msg.utime = int(round((time.time()-T0)*1000000))
#   input = math.sin(omega*((msg.utime+0.0)/1000000.0))
#   foto = np.array((input,0,0,    0,0,0))
#   fth = np.dot(tMc,foto)*500 + 3000*np.array((1,1,1,1,1,1))
#   #print fth

#   msg.servo_setpoints = (fth[0],fth[1],fth[2],fth[3],fth[4],fth[5])
#   lc.publish("NJORD_V2V_VEH1_SET_SERVOGROUP", msg.encode())
#   time.sleep(0.05)

# print fth
# End loopisms
