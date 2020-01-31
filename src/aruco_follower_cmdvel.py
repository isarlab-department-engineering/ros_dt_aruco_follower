#!/usr/bin/env python
from __future__ import print_function
import rospy, sys, cv2, time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from master_node.msg import *
from master_node.srv import *
from fiducial_msgs.msg import *


#impostare il nome del nodo coerente con quello del master
id_node = "aruco"

#impostare la risposta positiva coerente con quella del master
positive_answ = 1


I = 0
I_x = 0

last_error = 0
last_error_x = 0

setpoint_z = 0.3
setpoint_x = 0.0

last_speed = 0

max_speed = 180

twistMessage = Twist()
twistMessage.linear.x = 0
twistMessage.linear.y = 0
twistMessage.linear.z = 0
twistMessage.angular.x = 0
twistMessage.angular.y = 0
twistMessage.angular.z = 0


followmessage = Follow()

followmessage.id = id_node

lock = False


pub = rospy.Publisher("follow_topic",Follow,queue_size=1)
request_lock_service = rospy.ServiceProxy('request_lock',RequestLockService)
release_lock_service = rospy.ServiceProxy('release_lock',ReleaseLockService)


def calculatePID(error,Kp,Ki,Kd):
	global last_error, I
	
	P = error
	if P > 100:
		P = 100
	elif P < -100:
		P = -100

	I = I + error
	
	if I > 35:
		I = 35
	elif I < -35:
		I = -35



	D = error - last_error

	PID = float(Kp*P + Ki*I + Kd*D)

	last_error = error
	
	return PID


def calculatePID_x(error_x,Kp,Ki,Kd):
	global last_error_x, I_x
	
	P = error_x
	if P > 100:
		P = 100
	elif P < -100:
		P = -100

	I_x = I_x + error_x
	
	if I_x > 10:
		I_x = 10
	elif I_x < -10:
		I_x = -10

	#if error_x * I_x < 0:
		#I_x = 0

	D = error_x - last_error_x

	PID = float(Kp*P + Ki*I_x + Kd*D)

	last_error_x = error_x

	print('Integrale = ' , I_x)
	
	return PID



def turnOffMotors():
	# DEBUG VERSION TO FIX
	twistMessage.linear.x = 0
	twistMessage.angular.z = 0
	pub.publish(followmessage)

def setSpeed(x_speed,z_speed):
	if x_speed == 0 and z_speed == 0:
		turnOffMotors()
	else:
		twistMessage.linear.x = x_speed
		twistMessage.angular.z = z_speed
		followmessage.twist = twistMessage
		pub.publish(followmessage)

def callback(data):
	global setpoint_x, setpoint_z, last_speed, max_speed

	
	transform_array = data.transforms

	if transform_array == []:
		y = 0
		#print('non vedo nulla')
	else:
	
		transform = transform_array[0].transform
		vector = transform.translation

		current_x = vector.x
		current_z = vector.z

		error_x = (setpoint_x - current_x)
		error_z = (setpoint_z - current_z) #errore negativo sono troppo lontano

		print('errore z: ',error_z)
		print('errore x: ',error_x)




		PID_z = calculatePID(error_z,1,0,0)
		#PID_z = 0
		PID_x = calculatePID_x(error_x,1,0,0)
		#PID_x = np.abs(PID_x)
		#print ('PID_z: ', PID_z)
		print ('PID_x: ', PID_x)

		
		x_speed = -PID_z
		z_speed = PID_x

		print("X Speed: ", x_speed, "    Z_Speed: ", z_speed)
	
		setSpeed(x_speed, z_speed)

#		if error_z == 0:
#			
#			setSpeed(x_speed,x_speed)
#
#		elif (error_x > 0 and error_x < 150):
#			newSpeed1 = speed1-PID_x 
#			newSpeed2 = speed2+PID_x 
#			setSpeed(newSpeed1,newSpeed2)
#			
#
#		elif (error_x < 0):
#			newSpeed1 = speed1-PID_x 
#			newSpeed2 = speed2+PID_x 
#			setSpeed(newSpeed1,newSpeed2)
#
#		else:
#			turnOffMotors()
	

def aruco_follower():
	rospy.init_node('aruco_follower', anonymous=True)

    	#Sottosrizione al topic shared lock
    	rospy.Subscriber("lock_shared",Lock,checkMessage)


	rospy.Subscriber('fiducial_transforms', FiducialTransformArray, requestLock)
	try:
		rospy.on_shutdown(releaseLock)
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")


def releaseLock():
    global id_node, lock
    resp = release_lock_service(id_node)
    lock = False
    print(resp)


def requestLock(data):
    global id_node, lock
    if lock:
        callback(data)
    else:
        resp = request_lock_service(id_node)
        print(resp)
        if resp:
            lock = True
            callback(data)
        else:
            rospy.loginfo("SONO QUI")
            msg_shared = rospy.wait_for_message("/lock_shared", Lock)
            checkMessage(msg_shared)




def checkMessage(data):
    global id_node, lock
    if data.id == id_node:
        if data.msg == 1:
            lock = True
        else:
            lock = False
    else:
        msg_shared = rospy.wait_for_message("/lock_shared", Lock)
        checkMessage(msg_shared)



if __name__ == '__main__':
	aruco_follower()
