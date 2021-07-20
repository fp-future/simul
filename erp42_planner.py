#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys,os, math
import rospy
import rospkg
import numpy as np
from nav_msgs.msg import Path,Odometry
from std_msgs.msg import Float64,Int16,Float32MultiArray
from geometry_msgs.msg import PoseStamped,Point
from morai_msgs.msg import ERP42Info,ObjectStatusList,CtrlCmd,GetTrafficLightStatus,SetTrafficLight, EventInfo
from visualization_msgs.msg import MarkerArray
from lib.utils import Waypoint, pathReader, findLocalPath, Scenario
from lib.utils import purePursuit,cruiseControl,vaildObject,pidController,velocityPlanning,latticePlanner
from lib.utils import Recorder

from morai_msgs.srv import MoraiEventCmdSrv

import tf
from math import cos,sin,sqrt,pow,atan2,pi
import time

status = {
    'delivery' : False,
    'parking' : False,
    'record' : False,
    'replay' : False
}



class erp_planner():
    def __init__(self):
      '''
      /global_path, /local_path, /ctrl_cmd, /SetTrafficLight, /waypoints, lattice_path_{i} 발행
      /Ego_topic, /Object_topic, /GetTrafficLightStatus 구독
      대부분의 기능은 클래스화 되어 utils.py에 저장되어 있음.
      
      상태가 'parking'(주차)이면 현 위치와 park_waypoint 사이의 거리 측정
      지역 경로는 utils.py의 findLocalPath를 이용하여 설정
      장애물 정보를 가져와 전역 장애물과 지역 장애물을 설정
      utils.py의 latticePlanner을 이용하여 최종 경로 설정
      신호등이 있으면, 신호 종류(빨간불, 파란불)를 발행한 후 장애물 확인
      utils.py의 purePursuit을 이용하여 지역 경로에 따른 회전값 설정
      utils.py의 acc를 이용하여 구한 목표 속도값으로 PID 제어 후 컨트롤 메세지 발행
      시나리오가 끝나면 종료
      경로가 끝나면 주차를 하거나 다음 경로를 실행함.
      '''
        rospy.init_node('erp42_total', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name=arg[1]
        self.traffic_control=arg[2]

        #발행목록
        global_path_pub= rospy.Publisher('/global_path',Path, queue_size=1)
        local_path_pub= rospy.Publisher('/local_path',Path, queue_size=1)
        ctrl_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        traffic_pub = rospy.Publisher("/SetTrafficLight",SetTrafficLight,queue_size=1)

        waypoints_pub = rospy.Publisher('/waypoints',MarkerArray, queue_size=1)

        ########################  lattice  ########################
        lattice_branch = 31

        for i in range(1,lattice_branch + 1):  # Sangyoon 8 -> 10           
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  

        ########################  lattice  ########################

        ctrl_msg= CtrlCmd()
        
        #subscriber
        rospy.Subscriber("/Ego_topic", ERP42Info, self.statusCB)
        rospy.Subscriber("/Object_topic", ObjectStatusList, self.objectInfoCB) ## Object information Subscriber
        rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.getTL_callback)

        #def
        self.is_status=False
        self.is_obj=False
        self.is_traffic=False
        self.traffic_info = [[58.50, 1180.41 ,'C119BS010001'], ##신호등 정보(global_x, global_y, index)
                             [85.61, 1227.88 ,'C119BS010021'],
                             [136.58,1351.98 ,'C119BS010025'], # 원본은 26. issue #47
                             [141.02,1458.27 ,'C119BS010028'],
                             [139.39,1596.44 ,'C119BS010033'],
                             [48.71, 1208.02 ,'C119BS010005'],
                             [95.58, 1181.56 ,'C119BS010047'],
                             [104.46,1161.46 ,'C119BS010046'],
                             [85.29, 1191.77 ,'C119BS010007'],
                             [106.32,1237.04 ,'C119BS010022'],
                             [75.34, 1250.43 ,'C119BS010024'],
                             [73.62, 1218.01 ,'C119BS010012'],
                             [116.37,1190.65 ,'C119BS010040'],
                             [153.98,1371.48 ,'C119BS010073'],
                             [129.84,1385.08 ,'C119BS010039'],
                             [116.28,1367.77 ,'C119BS010074'],
                             [75.08, 1473.34 ,'C119BS010069'], # error default =  119BS010075
                             [67.10, 1506.66 ,'C119BS010076'],
                             [114.81,1485.81 ,'C119BS010079'],
                             [159.11,1496.63 ,'C119BS010060'],
                             [122.24,1608.26 ,'C119BS010072'],
                             [132.70,1624.78 ,'C119BS010034']]
        self.road_type = "straight"
        road_spawn_gap = 100

        #class
        path_reader=pathReader('erp_ros')
        pure_pursuit=purePursuit()
        self.cc=cruiseControl(0.5,1)
        self.vo=vaildObject(self.traffic_info)
        pid=pidController()       
        self.wp = Waypoint('kcity')
        self.rcd = Recorder()
        self.scenario = Scenario('wayp0int', 'scenario_210719_1148')
        
        debug = True

        #read path
        scenario = [
        #'P1', 'Aa5', 'Aae1' ,
        #'P1', 'P2', 'Aa12', 'Aad1', 'Aad2', 
            #'Aad3', 'Ad4', 'Adb', 'Ab4', 'Abe', 
            #'Ae5', 'Aec1', 'Aec2', 'Aec3', 'Ac7', 
            #'AB1', 'AB2', 'AB3', 'AB4', 'AB5', 'Ba1', 
            #'Bab1', 'Bab2', 'Bab3-1', 'Bb5', 'Bbe2', 
            #'Be5', 'Bec1-1', 'Bec2', 'Bec3', 'Bec4',
            #'Bc4', 'Bcb', 'Bcb1-1', 'Bcb1-2',
            #'Bb9',
             'P1', #'P2', 'Aa5', 'Aae1', 'Aae2', 'Aae3', 'Ae4', 'Aeb', 'Ab3', 'Abd', 'Ad5', 'Adc1', 'Adc2', 'Adc3', 'Ac10',
            # 'AB1', 'AB2', 'AB3', 'AB4', 'AB5', 'Ba1', 'Bab1', 'Bab2', 'Bab3-1', 'Bb5', 'Bbe3', 'Be3', 'Bea1', 'Bea2', 'Bea3', 'Ba6',
            # 'BA1', 'BA2', 'BA3', 'BA4', 'BA5', 'Ac2', 'Ab2', 'Aa2', 'P3'
        ]
        
        #self.global_paths = self.wp.get_global_path(scenario)
        self.global_paths = self.scenario.to_waypoints()
        self.global_path = self.global_paths[0]['path']

        #scenario = ['kcity']
        #self.global_paths = path_reader.read_paths(scenario)
        #self.global_path = self.global_paths[0]
        

        #self.path_name = "test"
        #self.global_path=path_reader.read_txt(self.path_name+".txt")

        #print(self.global_path)

        self.object_info_msg=ObjectStatusList()
        self.status_msg=ERP42Info()

        vel_planner=velocityPlanning(200/3.6,1.5)
        vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)

        #time var
        count=0
        rate = rospy.Rate(30) # 30hz

        lattice_current_lane=3

        self.current_waypoint = 0

        park_waypoints = [
            []
        ]

        #메인 실행
        while not rospy.is_shutdown():
            if self.is_status==True  and self.is_obj==True:
                #주차 상태
                if status['parking']:
                    for park_waypoint in park_waypoints:
                        max_term = 1
                        x = self.status_msg.position_x
                        y = self.status_msg.position_y

                        distance = ((x - park_waypoint[0]) ** 2 + (y - park_waypoint[1]) ** 2) ** 0.5

                        if distance < max_term:
                            #stop
                            continue
                
                #지역경로
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg)
                
                #전역경로
                self.vo.get_object(self.object_num,self.object_info[0],self.object_info[1],self.object_info[2],self.object_info[3])
                global_obj,local_obj=self.vo.calc_vaild_obj([self.status_msg.position_x,self.status_msg.position_y,self.status_msg.yaw/180*pi])

                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position_x,self.status_msg.position_y,(self.status_msg.yaw+90)/180*pi,self.status_msg.velocity/3.6]
                lattice_path,selected_lane=latticePlanner(
                    local_path,global_obj,vehicle_status,lattice_current_lane,
                    lattice_branch = lattice_branch,
                    lattice_gap = 0.2
                )
                lattice_current_lane=selected_lane
                                
                if selected_lane != -1: 
                    local_path=lattice_path[selected_lane]                
                
                if len(lattice_path)==lattice_branch:                  
                    for i in range(1,lattice_branch + 1):
                        globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################
                if self.is_traffic == True:
                    if self.traffic_control == "True":
                        self.tl_msg.trafficLightStatus=16
                        ###################### traffic_control ######################
                        self.set_traffic_data= SetTrafficLight()
                        self.set_traffic_data.trafficLightIndex = self.tl_msg.trafficLightIndex
                        self.set_traffic_data.trafficLightStatus = 16 ##set greenlight 
                        traffic_pub.publish(self.set_traffic_data)

                    self.cc.checkObject(local_path,global_obj,local_obj,[self.tl_msg.trafficLightIndex,self.tl_msg.trafficLightStatus])

                else :
                    self.cc.checkObject(local_path,global_obj,local_obj)
                
                #pure pursuit
                pure_pursuit.getPath(local_path)
                pure_pursuit.getEgoStatus(self.status_msg)

                ctrl_msg.steering=pure_pursuit.steering_angle()

                #acc
                cc_vel = self.cc.acc(local_obj,self.status_msg.velocity,vel_profile[self.current_waypoint],self.status_msg)
                
                target_velocity = cc_vel
                ctrl_msg.velocity=cc_vel
                

                #pid
                control_input=pid.pid(target_velocity,self.status_msg.velocity) ## 속도 제어를 위한 PID 적용 (target Velocity, Status Velocity)
                
                if control_input > 0 :
                    ctrl_msg.accel= control_input
                    ctrl_msg.brake= 0

                else :
                    ctrl_msg.accel= 0
                    ctrl_msg.brake= -control_input

                if self.status_msg.velocity < 3.0  and target_velocity<=0.0:
                    ctrl_msg.accel=0
                    ctrl_msg.brake=1
                
                '''
                if self.road_type == "parking":
                    self.status_msg.velocity /= 3
                '''

                local_path_pub.publish(local_path)
                ctrl_pub.publish(ctrl_msg)
                
            if count/300==1 :
                #waypoints_pub.publish(self.scenario.get_rviz_data())
                global_path_pub.publish(self.global_path)
                count=0

            count+=1
            self.steering_angle=ctrl_msg.steering

            if debug and count > 5 and count % 5 == 0:
                self.print_info()
                pass
            
            # 시나리오 및 경로 확인
            is_scenario_end = (len(self.global_paths) <= 1)
            is_path_end = ((len(self.global_path.poses) - self.current_waypoint) < road_spawn_gap)
            if is_scenario_end:
                # 시나리오가 끝나면 무조건 멈춰!
                #print("멈춰!")
                evt_info = EventInfo()
                evt_info.option = 2
                evt_info.gear = 1 # P
                #apply = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)
                #apply()
                pass
            else:
                #경로 끝날 시 주차 혹은 다음 경로 
                if is_path_end:
                    '''
                    print("path end!!__", self.road_type)
                    # 주차 미션의 경우 정차 후 빠져나옴.
                    if self.road_type == "parking":
                        road_spawn_gap = 20
                        print("parking end!!")
                        evt_info = EventInfo()
                        evt_info.option = 2
                        evt_info.gear = 1 # P
                        # publish
                        #apply = rospy.ServiceProxy('/Service_MoraiEventCmd', EventInfo)
                        #apply(evt_info)
                        math.sleep(10)
                        evt_info.option = 2
                        evt_info.gear = 2 # R
                        
                        # publish
                        #apply = rospy.ServiceProxy('/Service_MoraiEventCmd', EventInfo)
                        #apply(evt_info)
                        # replay waypoint
                        ctrl_msgs = self.rcd.replay_list()
                        for ctrl_msg in ctrl_msgs:
                            ctrl_pub.publish(ctrl_msg)
                        
                        evt_info.gear = 2 # D
                        road_spawn_gap = 100
                    '''
                    # 다음 시나리오로 변경.
                    self.global_paths = self.global_paths[1:]
                    #self.road_type = self.global_paths[0]['type']
                    self.global_path.poses += self.global_paths[0]['path'].poses
                    vel_profile=vel_planner.curveBasedVelocity(self.global_path,100)
                    global_path_pub.publish(self.global_path)

                

            rate.sleep()


    def print_info(self):
      '''
      메인 실행이 되는 상태를 터미널에 표시
      '''
        os.system('clear')
        print('--------------------status-------------------------')
        print('position :{0} ,{1}, {2}'.format(self.status_msg.position_x,self.status_msg.position_y,self.status_msg.position_z))
        print('velocity :{} km/h'.format(self.status_msg.velocity))
        print('heading :{} deg'.format(self.status_msg.yaw-90))

        print('--------------------controller-------------------------')
        print('target steering_angle :{} deg'.format(self.steering_angle))

        print('--------------------localization-------------------------')
        print('all waypoint size: {} '.format(len(self.global_path.poses)))
        print('current waypoint : {} '.format(self.current_waypoint))

        if self.is_traffic==True:
            print('--------------------trafficLight-------------------------')        
            print('traffic mode : {}'.format(self.tl_msg.isAutoMode))
            print('traffic index : {}'.format(self.tl_msg.trafficLightIndex))
            print('traffic type : {}'.format(self.tl_msg.trafficLightType))
            print('traffic status : {}'.format(self.tl_msg.trafficLightStatus))


    '''
    아래는 콜백함수로 subscriber(구독)후 구독한 정보로 메인이 실행되기 전 전처리함수.
    '''
    def statusCB(self,data):
      '''
      좌표값을 tf(좌표변환)함.
      yaw값을 오일러 좌표계에서 쿼터니언 좌표계로 변환.
      '''
        self.status_msg=ERP42Info()
        self.status_msg=data
        br = tf.TransformBroadcaster()
        br.sendTransform((self.status_msg.position_x, self.status_msg.position_y, self.status_msg.position_z),
                        tf.transformations.quaternion_from_euler(0, 0, self.status_msg.yaw/180*pi),
                        rospy.Time.now(),
                        "gps",
                        "map")
        self.is_status=True                 

    def objectInfoCB(self,data):
      '''
      장애물을 npcs, obstacle, pedestrain으로 분류하여 각각의 x, y, velocity값을 저장
      '''
        self.object_num=data.num_of_npcs+data.num_of_obstacle+data.num_of_pedestrian
        object_type=[]
        object_pose_x=[]
        object_pose_y=[]
        object_velocity=[]
        for num in range(data.num_of_npcs) :
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity)

        for num in range(data.num_of_obstacle) :
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity)

        for num in range(data.num_of_pedestrian) :
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity)

        self.object_info=[object_type,object_pose_x,object_pose_y,object_velocity]
        self.is_obj=True

    def getTL_callback(self,msg):
      '''
      신호등 정보를 받아와 신호등 상태 파악
      '''
        self.is_traffic=True
        self.tl_msg=GetTrafficLightStatus()
        self.tl_msg=msg
        

    
if __name__ == '__main__':
    try:
        kcity_pathtracking=erp_planner()
    except rospy.ROSInterruptException:
        pass
