#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3
import threading
import math
import datetime

class UAV:
    def __init__(self,id=""):
        self.uav_id=id


    def thread_offboard(self):
        rospy.init_node('offboard', anonymous=True)
        rate=rospy.Rate(30)
        pub=rospy.Publisher(self.uav_id+"/cmd/posref",Vector3,queue_size=10)
        #dt=0.01
        now=rospy.get_rostime()
        init_request=now.to_sec() 
        msg=Vector3()
        lst=datetime.datetime.now()
        while not (rospy.is_shutdown()):
            now=rospy.get_rostime().to_sec()
            tm=now-init_request
            #while (datetime.datetime.now()-lst).total_seconds()<dt:
            #    pass
            #print (datetime.datetime.now()-lst).total_seconds()
            lst=datetime.datetime.now()
            
            if(now-init_request<3):
                msg.z=min(tm/2,1)
                msg.x=0
                msg.y=0
            elif(tm<6):
                msg.z=1
                msg.x=min((tm-3)/2,1)
            elif(tm<12.28):
                msg.z=1
                msg.x=math.cos(tm-6)
                msg.y=math.sin(tm-6)
            elif(tm<15):
                msg.z=1
                msg.x=max(1-(15-12.28)/2,0)
                msg.y=0
            elif(tm<16):
                msg.z=16-tm
                msg.x=0
                msg.y=0
            else:
                msg.z=-100
                msg.x=0
                msg.y=0
            rospy.loginfo("Position Ref:[%.2f,%.2f,%.2f]"%(msg.x,msg.y,msg.z))
            pub.publish(msg)
           
            
            rate.sleep()

if __name__ == '__main__':
    uav=UAV()
    uav.thread_offboard()
