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

setpoint_z = 0.8
setpoint_x = -0.25

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

	PID = int(Kp*P + Ki*I + Kd*D)

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

	PID = int(Kp*P + Ki*I_x + Kd*D)

	last_error_x = error_x

	print('Integrale = ' , I_x)
	
	return PID



def turnOffMotors():
	# DEBUG VERSION TO FIX
	twistMessage.linear.x = 0
	twistMessage.linear.y = 0
	pub.publish(followmessage)

def setSpeed(speed1,speed2):
	if speed1 == 0 and speed2 == 0:
		turnOffMotors()
	else:
		twistMessage.linear.x = speed1
		twistMessage.linear.y = speed2
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

		error_x = (setpoint_x - current_x)*100
		error_z = (setpoint_z - current_z)*100 #errore negativo sono troppo lontano

		print('errore z: ',error_z)
		print('errore x: ',error_x)




		PID_z = calculatePID(error_z,1.5,0.15,0.1)
		#PID_z = 0
		PID_x = calculatePID_x(error_x,0.6,0.5,0.4)
		#PID_x = np.abs(PID_x)
		#print ('PID_z: ', PID_z)
		print ('PID_x: ', PID_x)


		speed2 = -PID_z
		motorBalance = 0
		speed1 = speed2 + motorBalance


		if error_z == 0:
			
			setSpeed(speed1,speed2)

		elif (error_x > 0 and error_x < 150):
			newSpeed1 = speed1-PID_x 
			newSpeed2 = speed2+PID_x 
			setSpeed(newSpeed1,newSpeed2)
			

		elif (error_x < 0):
			newSpeed1 = speed1-PID_x 
			newSpeed2 = speed2+PID_x 
			setSpeed(newSpeed1,newSpeed2)

		else:
			turnOffMotors()
	

def aruco_follower():
	rospy.init_node('aruco_follower', anonymous=True)

    	#Sottosrizione al topic shared lock
    	rospy.Subscriber("lock_shared",Lock,checkMessage)


	rospy.Subscriber('fiducial_transforms', FiducialTransformArray, requestLock)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")



    #TODO RELEASE LOCK 

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
