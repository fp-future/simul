#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from morai_msgs.msg  import ERP42Info
from math import pi,cos,sin,pi,sqrt,pow
from nav_msgs.msg import Path
import tf
from geometry_msgs.msg import PoseStamped


class test :
  '''
  경로를 저장할 파일을 생성
  현재 좌표와 이전 좌표의 거리를 측정하여 0.3m 초과이면 좌표값 파일에 저장.
  '''

    def __init__(self):
      '''
      /Ego_topic 구독 후 status_callback 호출
      /global_path 발행
      
      erp_ros 패키지 경로/경로 폴더명/생성경로명.txt 파일 생성 후 path_make 함수 반환값 저장
      '''
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_folder_name=arg[1]
        self.make_path_name=arg[2]
        

        rospy.Subscriber("/Ego_topic",ERP42Info, self.status_callback)
        self.global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)

        self.is_status=False
        self.prev_x = 0
        self.prev_y = 0

        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('erp_ros')
        full_path=pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f=open(full_path, 'w')

        rate=rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_status==True :
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
      '''
      차량의 x, y 값을 받아 이전 좌표와 거리 측정
      거리가 0.3m 초과일 시 파일에 좌표값 작성
      '''
        x=self.status_msg.position_x
        y=self.status_msg.position_y
        z=self.status_msg.position_z
        distance=sqrt(pow(x-self.prev_x,2)+pow(y-self.prev_y,2))
        if distance > 0.3:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print(x,y)

    def status_callback(self,msg): ## Vehicl Status Subscriber 
      '''
      x, y, z, yaw값을 좌표변환(tf)을 이용하여 변형 후 광고
      주로 yaw 값을 오일러 좌표계에서 쿼터니언 좌표계로 변환.
      '''
        self.is_status=True
        self.status_msg=msg
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position_x, self.status_msg.position_y, self.status_msg.position_z),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.yaw+90)/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        

if __name__ == '__main__':
    try:
        test_track=test()
    except rospy.ROSInterruptException:
        pass
