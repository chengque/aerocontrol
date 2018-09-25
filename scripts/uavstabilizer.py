#!/usr/bin/env python
import rospy
from std_msgs.msg import String,Float64,Float64MultiArray
from uavinstance import UAV
import tf_conversions
from geometry_msgs.msg import PoseStamped,TwistStamped,Quaternion,Vector3
from viconros.msg import viconmocap
import matplotlib.pyplot as plt
from Control import PIDUtility
import numpy as np
import time
import math
import copy
from Predictor import *
from collections import deque



class stabilizer:
	def __init__(self,id=""):
		self.uav_id=id
		self.position=Vector3()
		self.velocity=Vector3()
		self.ppidx=PIDUtility(self.position.x)
		self.vpidx=PIDUtility(self.velocity.x)

		self.ppidy=PIDUtility(self.position.y)
		self.vpidy=PIDUtility(self.velocity.y)

		self.ppidz=PIDUtility(self.position.z)
		self.vpidz=PIDUtility(self.velocity.z)

		self.getfeed=False


	def poscallback(self,data):
		self.att=tf_conversions.transformations.euler_from_quaternion([data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w])


	def viconcallback(self,data):
		self.position=data.position	
		self.velocity=data.velocity


	def poscontrolx(self,parm,dr,x,dt,lim,ddx=0):
		vr=self.ppidx.iteratewithcorrection(parm[0],x[0],dr,ddx,dt)
		#vr=self.ppidx.iterate(parm[0],x[0],dr,dt)
		vlim=5
		if vr>vlim:
			vr=vlim
		if vr<-vlim:
			vr=-vlim


		tr=self.vpidx.iterate(parm[1],x[1],vr,dt)
		#tr=self.vpidx.iteratewithcorrection(parm[1],x[1],vr,ddx,dt)
		if tr>lim[1]:
			tr=lim[1]
		if tr<lim[0]:
			tr=lim[0]
		return vr,tr



	def poscontroly(self,parm,dr,x,dt,lim,ddx=0):
		vr=self.ppidy.iterate(parm[0],x[0],dr,dt)
		vlim=5
		if vr>vlim:
			vr=vlim
		if vr<-vlim:
			vr=-vlim

		tr=self.vpidy.iteratewithcorrection(parm[1],x[1],vr,ddx,dt)
		if tr>lim[1]:
			tr=lim[1]
		if tr<lim[0]:
			tr=lim[0]
		return vr,tr

	def poscontrolz(self,parm,dr,x,dt,lim):
		vr=self.ppidz.iterate(parm[0],x[0],dr,dt)
		vlim=5
		if vr>vlim:
			vr=vlim
		if vr<-vlim:
			vr=-vlim


		tr=self.vpidz.iterate(parm[1],x[1],vr,dt)+0.5
		if tr>lim[1]:
			tr=lim[1]
		if tr<lim[0]:
			tr=lim[0]
		return vr,tr


	def controlonce(self,throt,att):
		rospy.init_node('offboard_control', anonymous=True)
		self.uav_id=rospy.get_param("~id","")
		rospy.loginfo(self.uav_id+" master:start offboard control..")
		rate=rospy.Rate(30)
		rate.sleep()

		pub=rospy.Publisher(self.uav_id+"/cmd/throttle", Float64,queue_size=10)
		pubrr=rospy.Publisher(self.uav_id+"/cmd/ref_pos", Vector3,queue_size=10)
		pubvr=rospy.Publisher(self.uav_id+"/cmd/ref_vel", Vector3,queue_size=10)
		pubva=rospy.Publisher(self.uav_id+"/cmd/ref_att", Vector3,queue_size=10)
		pub1=rospy.Publisher(self.uav_id+"/cmd/orentation", Quaternion,queue_size=10)

		rospy.Subscriber(self.uav_id+"/mavros/local_position/pose", PoseStamped, self.poscallback)
		rospy.Subscriber(self.uav_id+"vicon", viconmocap, self.viconcallback)
		time.sleep(5)
		
		now=rospy.get_rostime()
		last_request=now.to_sec()-0.02
		ref_pos=Vector3()
		ref_vel=Vector3()
		ref_att=Vector3()
		parm=np.zeros((2,3))
		i=0
		itime=rospy.get_rostime().to_sec()

		while not (rospy.is_shutdown()):
			now=rospy.get_rostime().to_sec()-itime
			dt=now-last_request
			last_request=now
			if(dt<0.01):
				dt=0.01
			parm[0]=[2.3,0,0]
			parm[1]=[0.31,0.01,0]
			ref_pos.z=0.5
			i=i+1
			if(i>10000):
				i=10000
			if(now<10):
				ref_pos.x=0
				ref_pos.y=0
			else:
				ref_pos.x=0
				ref_pos.y=0
				ref_pos.z=0.0

			vz,tr=self.poscontrolz(parm,ref_pos.z,[self.position.z,self.velocity.z],dt,[0.1,0.7])

			vx,xr=self.poscontrolx(parm,ref_pos.x,[self.position.x,self.velocity.x],dt,[-0.4,0.4])
			vy,yr=self.poscontroly(parm,ref_pos.y,[self.position.y,self.velocity.y],dt,[-0.4,0.4])
			ref_vel.x=vx
			ref_vel.y=vy
			ref_vel.z=vz
			
			pub.publish(tr)
			pubvr.publish(ref_vel)
			pubrr.publish(ref_pos)

			q=tf_conversions.transformations.quaternion_from_euler(-yr,xr,0)
			ref_att.x=-yr
			ref_att.y=xr
			pubva.publish(ref_att)

			self.qx.popleft()
			self.qx.append(ref_att.x)
			self.qy.popleft()
			self.qy.append(ref_att.y)
			self.qt.popleft()
			self.qt.append(tr)

			quat=Quaternion()
			quat.x=q[0]
			quat.y=q[1]
			quat.z=q[2]
			quat.w=q[3]
			pub1.publish(quat)
			rate.sleep()



if __name__ == '__main__':
	uav=stabilizer()
	#uav.control(0.5,[0,0,0])
	uav.controlonce(0.5,[0,0,0])
	#uav.controlcorr(0.5,[0,0,0])
	#uav.controlonce(0.5,[0,0,0])
