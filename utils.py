#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rospkg
import math
from nav_msgs.msg import Path,Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import Float64,Int16,Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi
import tf
import json
import os
import pickle

class Waypoint:
    def __init__(self, map_file_name):
        '''
        미리 작성한 map파일 로드.
        파일 경로 = 'erp_ros'경로/path/지도 파일명.json
        '''
        rospack=rospkg.RosPack()
        json_path = rospack.get_path("erp_ros") + '/path/' + map_file_name + '.json'
        self.roads = json.loads(open(json_path).read())

    def get_global_path(self, scenario):
        '''
        반환값 : global_path로 만든 waypoints
        scenario의 타입에 따라 직선, 곡선 등 waypoint를 분리해서 좌표값(pose)으로 변환 후 최종 global_path를 만든다.
        '''
        self.out_path = Path()
        self.out_path.header.frame_id='/map'

        self.waypoints = []
        for name in scenario:
            road = self.roads[name]
            if road['type'] == 'straight':
                sub_waypoints = self._calc_straight_sub_waypoint(
                    road['waypoints'][0],
                    road['waypoints'][1]
                )
                self.waypoints.append({
                    'type': 'straight',
                    'waypoints': sub_waypoints
                })
                
                
            elif road['type'] == 'curve':
                sub_waypoints = self._calc_curved_sub_waypoint(
                    road['waypoints'][0],
                    road['waypoints'][1],
                    road['waypoints'][2]
                )
                self.waypoints.append({
                    'type': 'curve',
                    'waypoints': sub_waypoints
                })

            elif road['type'] == 'parking':
                sub_waypoints = self._calc_straight_sub_waypoint(
                    road['waypoints'][0],
                    road['waypoints'][1]
                )
                self.waypoints.append({
                    'type': 'parking',
                    'waypoints': sub_waypoints
                })

            elif road['type'] == 'straight-end':
                sub_waypoints = self._calc_straight_sub_waypoint(
                    road['waypoints'][0],
                    road['waypoints'][1]
                )
                if len(road['waypoints']) > 2:
                    sub_waypoints += self._calc_straight_sub_waypoint(
                        road['waypoints'][1],
                        road['waypoints'][2]
                    )

                self.waypoints.append({
                    'type': 'straight',
                    'waypoints': sub_waypoints
                })

        self._convert_waypoints_to_poses()

        return self.waypoints

    def get_rviz_data(self):
        '''
        반환값 : markerArray
        Rviz에서 사용되는 markerArray를 제작 후 반환.
        frame_id = /map
        x, y, z 표시 마커의 크기 및 색깔 설정.
        x, y, z는 각각 waypoint 0, 1, 2에 할당한다.
        '''
        markerArray = MarkerArray()

        for road_no, road in enumerate(self.waypoints):
            for waypoint_no, waypoint in enumerate(road['waypoints']):
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.id = road_no * 1000 + waypoint_no
                marker.type = marker.SPHERE
                marker.action = marker.ADD
                marker.scale.z = 0.2
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.color.a = 1.0
                marker.color.r = 0.8
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = waypoint[0]
                marker.pose.position.y = waypoint[1]
                marker.pose.position.z = waypoint[2]
                '''
                if waypoint_no == (len(road['waypoints']) - 1):
                    print("###############"*10)
                    text = Marker()
                    text.header.frame_id = "/map"
                    text.id = road_no * 1000 + waypoint_no + 100
                    text.type = marker.TEXT_VIEW_FACING
                    text.action = marker.ADD
                    text.scale.z = 0.1
                    text.scale.x = 0.3
                    text.scale.y = 0.3
                    text.color.a = 1.0
                    text.color.r = 0.3
                    text.color.g = 0.0
                    text.color.b = 0.0
                    text.pose.orientation.w = 1.0
                    text.pose.position.x = waypoint[0] + 0.001
                    text.pose.position.y = waypoint[1]
                    text.pose.position.z = waypoint[2]
                    text.text = road['name']
                    text.ns = road['name']
                    markerArray.markers.append(text)
                '''
                markerArray.markers.append(marker)

        return markerArray

    def _calc_straight_sub_waypoint(self, position1, position2):
        '''
        반환값 : 직선 도로 waypoints
        지정한 간격만큼 이웃한 두 점의 거리로 waypoint 개수 측정
        x, y 각각의 간격을 측정한 후 초기값에 대입하여 직선 waypoint 생성
        -> 이는 직선이기 때문에 x, y 중 1개만 변하는 점이 있기 때문이다.
        땅이 평평하다는 조건 하에 z 값 고려하지 않음
        '''
        waypoint_term = 0.3

        x1, y1, z1 = position1
        x2, y2, z2 = position2

        x_distance = x2-x1
        y_distance = y2-y1
        z_distance = z2-z1

        distance = (x_distance ** 2 + y_distance ** 2) ** 0.5
        num_of_waypoint = int(distance // waypoint_term)


        x_term = x_distance / num_of_waypoint
        y_term = y_distance / num_of_waypoint

        waypoints = []
        for i in range(num_of_waypoint):
            x = x1 + x_term * i
            y = y1 + y_term * i
            waypoints.append([x, y, z2])

        return waypoints

    def _calc_curved_sub_waypoint(self, position1, position2, position3):
        '''
        반환값 : 곡선 waypoint
        3개의 점을 이용하여 원의 방정식으로 곡선을 구한다.
        점 (1,2)/(2,3)의 거리와 기울기를 구함.
        이를 이용하여 x, y의 중심을 구함.
        x, y 중심으로 원의 반지름을 구하고, 앞서 구한 거리로 waypoint 개수를 구함.
        x, y 각각의 간격을 구하고 원의 방정식을 이용하여 최종 곡선 waypoint를 생성.
        '''
        waypoint_term = 0.3

        x1, y1, z1 = position1
        x2, y2, z2 = position2
        x3, y3, z3 = position3
        
        
        distance1 = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        distance2 = ((x3 - x2) ** 2 + (y3 - y2) ** 2) ** 0.5
        
        d1 = (x2 - x1) / (y2 - y1)
        d2 = (x3 - x2) / (y3 - y2)
        
        center_x = ((y3 - y1) + (x2 + x3) * d2 - (x1 + x2) * d1) / (2 * (d2 - d1))
        center_y = -1 * d1 * (center_x - (x1 + x2) / 2) + (y1 + y2) / 2
        
        radius = ((x1 - center_x) ** 2 + (y1 - center_y) ** 2) ** 0.5
        
        num_of_waypoint = int((distance1 + distance2) // waypoint_term)
        
        x_term = (x3 - x1) / num_of_waypoint
        y_term = (y3 - y1) / num_of_waypoint
        
        waypoints = self.waypoints[-1]['waypoints'][-2:]
        
        # 원방 : radius ** 2 == (x-x2) ** 2 + (y-y1) ** 2
        if abs(x_term) > abs(y_term):
            for i in range(num_of_waypoint):
                x = x1 + x_term * i
                y = (radius**2 - (x - center_x) ** 2) ** 0.5 # + center_y 

                #x, y = self._check_term_with_prev_point(waypoints[-1], [[x, y + center_y], [x, -1 * y + center_y]])
                x, y = self._check_term_with_end_point(position2, [[x, y + center_y], [x, -1 * y + center_y]])

                waypoints.append([x, y, z1])
        else:
            for i in range(num_of_waypoint):
                y = y1 + y_term * i
                x = (radius**2 - (y - center_y) ** 2) ** 0.5 # + center_x 

                #x, y = self._check_term_with_prev_point(waypoints[-1], [[x + center_x, y], [-1 * x + center_x, y]])
                x, y = self._check_term_with_end_point(position2, [[x + center_x, y], [-1 * x + center_x, y]])

                waypoints.append([x, y, z1])
        
        return waypoints[2:]

    def _check_term_with_end_point(self, end_point, current_points):
        '''
        반환값 : end point를 이용하여 간격 측정
        end point와 두점 사이의 거리값을 이용하여 이상치 조정.
        '''
        is_out_of_angle = False

        # 미래 position
        x1 = end_point[0]
        y1 = end_point[1]

        # 현재 position
        x2 = current_points[0][0]
        y2 = current_points[0][1]
        x3 = current_points[1][0]
        y3 = current_points[1][1]

        distance1 = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        distance2 = ((x3 - x1) ** 2 + (y3 - y1) ** 2) ** 0.5

        # print(distance1, distance2)

        less_distance_position = current_points[0]

        if distance1 > distance2:
            less_distance_position = current_points[1]
        
        return less_distance_position

    def _check_term_with_prev_point(self, prev_point, current_points):
        '''
        반환값 : 이전 point로 생성 waypoint 간격 측정
        3점 사이의 거리를 이용하여 waypoint 간격에 이상치가 생기지 않도록 조정
        '''
        is_out_of_angle = False

        # 과거 position
        x1 = prev_point[0]
        y1 = prev_point[1]

        # 현재 position
        x2 = current_points[0][0]
        y2 = current_points[0][1]
        x3 = current_points[1][0]
        y3 = current_points[1][1]

        distance1 = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        distance2 = ((x3 - x1) ** 2 + (y3 - y1) ** 2) ** 0.5

        # print(distance1, distance2)

        less_distance_position = current_points[0]

        if distance1 > distance2:
            less_distance_position = current_points[1]
        
        return less_distance_position
    
    def _check_angle_with_prev_point(self, prev_points, current_points):
        '''
        반환값 : 이전 point로 각도 조정
        4개의 점을 이용하여 기울기를 측정한 다음 tan^-1(=atan)을 이용하여 이상치가 생기지 않도록 조정
        이 코드에서는 사용하지 않음.
        '''
        is_out_of_angle = False

        # 과거 position
        x1 = prev_points[0][0]
        y1 = prev_points[0][1]
        x2 = prev_points[1][0]
        y2 = prev_points[1][1]

        # 현재 position
        x3 = current_points[0][0]
        y3 = current_points[0][1]
        x4 = current_points[1][0]
        y4 = current_points[1][1]

        prev_slope = abs((y1 - y2) / (x1 - x2))
        current_slope1 = abs((y1 - y3) / (x1 - x3))
        current_slope2 = abs((y1 - y4) / (x1 - x4))

        angle_gap1 = abs(math.atan2(prev_slope - current_slope1, 1 + prev_slope * current_slope1))
        angle_gap2 = abs(math.atan2(prev_slope - current_slope2, 1 + prev_slope * current_slope2))

        less_angle_position = current_points[0]

        if angle_gap1 > angle_gap2:
            less_angle_position = current_points[1]
        
        return less_angle_position


    def _convert_waypoints_to_poses(self):
        '''
        앞서 생성한 waypoint를 ros 메세지 형식으로 바꿈(x, y, z, yaw)
        '''
        paths = []
        for idx, load in enumerate(self.waypoints):
            path = load['waypoints']
            out_path = Path()
            out_path.header.frame_id='/map'
            for waypoint in path:
                read_pose=PoseStamped()
                read_pose.pose.position.x=float(waypoint[0])
                read_pose.pose.position.y=float(waypoint[1])
                read_pose.pose.position.z=float(waypoint[2])
                read_pose.pose.orientation.x=0
                read_pose.pose.orientation.y=0
                read_pose.pose.orientation.z=0
                read_pose.pose.orientation.w=1
                out_path.poses.append(read_pose)
            self.waypoints[idx]['path'] = out_path

class Recorder:
    '''
    컨트롤 메세지를 저장하고 재생하는 클래스
    '''
    def __init__(self):
        self.ctrl_buffer = []
        self.pointer = -1
    
    def record_once(self, ctrl_msg):
        '''
        컨트롤 메세지를 받아 컨트롤 버퍼에 저장
        포인터 값은 1 증가
        '''
        self.ctrl_buffer.append(ctrl_msg)
        self.pointer += 1
    
    def replay_once(self):
        '''
        포인터 값을 이용하여 생성된 컨트롤 버퍼를 반환해줌(재생)
        '''
        if self.pointer >= 0:
            self.pointer -= 1
            return self.ctrl_buffer[self.pointer + 1]

    def replay_list(self):
        '''
        컨트롤 버퍼를 반환하여 재생 리스트 생성
        '''
        return self.ctrl_buffer

class Scenario:
    '''
    시나리오 파일을 읽어와 ROS 메세지 형식으로 global_path 추가
    '''
    def __init__(self, pkg_name, scenario_name):
        '''
        패키지명으로 파일경로 읽음.
        패키지경로/scenario/시나리오명.pickle 파일 열기
        '''
        rospack=rospkg.RosPack()
        self.pkg_path = rospack.get_path(pkg_name)
        self.file_path = self.pkg_path + '/scenario/' + scenario_name + '.pickle'
        self.scenario = pickle.load(open(self.file_path, 'rb')) # , encoding='bytes'
    
    def to_waypoints(self):
        '''
        시나리오에서 앞서 waypoint를 ros 메세지 형식으로 변환하여 global_path에 추가
        x, y, z, yaw
        '''
        self.global_paths = []
        for link in self.scenario:
            path=Path()
            path.header.frame_id='/map'
            for point in link['points'] :
                read_pose=PoseStamped()
                read_pose.pose.position.x=float(point[0])
                read_pose.pose.position.y=float(point[1])
                read_pose.pose.position.z=float(point[2])
                read_pose.pose.orientation.x=0
                read_pose.pose.orientation.y=0
                read_pose.pose.orientation.z=0
                read_pose.pose.orientation.w=1
                path.poses.append(read_pose)

            link['path'] = path
            self.global_paths.append(link)

        return self.global_paths

    def get_paths(self):
        return

class pathReader :
    '''
    지역경로를 읽어 경로에 맞는 waypoint 설정 후 ROS 메세지로 변환
    '''
    def __init__(self,pkg_name):
        '''
        패키지 경로 읽어오기
        '''
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        '''
        파일경로/path/파일명.txt 로 파일 읽어오기
        읽어온 파일의 x, y, z, yaw 값을 tmp에 저장
        '''
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='/map'
        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            read_pose=PoseStamped()
            read_pose.pose.position.x=float(tmp[0])
            read_pose.pose.position.y=float(tmp[1])
            read_pose.pose.position.z=float(tmp[2])
            read_pose.pose.orientation.x=0
            read_pose.pose.orientation.y=0
            read_pose.pose.orientation.z=0
            read_pose.pose.orientation.w=1
            out_path.poses.append(read_pose)
        
        
        openFile.close()
        return out_path
    
    def read_paths(self, scenario):
        '''
        시나리오 상에 있는 경로명으로 경로 읽기
        '''
        paths = []
        for path_name in scenario:
            paths.append(self.read_txt(path_name+".txt"))
        
        return paths

def findLocalPath(ref_path,status_msg):
    '''
    지역 경로 설정하기
    현재 위치를 인식하고, ref_path(목표 경로)의 위치와 거리 측정
    측정된 거리에 따라 지역 waypoint 설정
    지역 waypoint를 ROS 메세지 형식으로 변환
    '''
    out_path=Path()
    current_x=status_msg.position_x
    current_y=status_msg.position_y
    current_waypoint=0
    min_dis=float('inf')

    for i in range(len(ref_path.poses)) :
        dx=current_x - ref_path.poses[i].pose.position.x
        dy=current_y - ref_path.poses[i].pose.position.y
        dis=sqrt(dx*dx + dy*dy)
        if dis < min_dis :
            min_dis=dis
            current_waypoint=i


    if current_waypoint+50 > len(ref_path.poses) :
        last_local_waypoint= len(ref_path.poses)
    else :
        last_local_waypoint=current_waypoint+50



    out_path.header.frame_id='map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose=PoseStamped()
        tmp_pose.pose.position.x=ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y=ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z=ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x=0
        tmp_pose.pose.orientation.y=0
        tmp_pose.pose.orientation.z=0
        tmp_pose.pose.orientation.w=1
        out_path.poses.append(tmp_pose)

    return out_path,current_waypoint

class velocityPlanning :
    '''
    전역 경로를 통해 정해진 개수(point_num)만큼 속도값 생성
    '''
    def __init__(self,car_max_speed,road_friction):
        '''
        차량 최대 속도와 마찰력 설정
        '''
        self.car_max_speed=car_max_speed
        self.road_friction=road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        '''
        0 ~ point_num(설정값) : 차량 최고 속도 입력
        point_num ~ {전역경로 waypoint 개수 - point_num} : 행렬 계산을 통해 최고 속도 구한 후 값 입력
        {전역경로 waypoint 개수 - point_num} ~ 끝 : 차량 최고 속도 입력
        위의 방식으로 속도값 저장 후 반환
        '''
        out_vel_plan=[]
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path.poses)-point_num):
            x_list=[]
            y_list=[]
            for box in  range(-point_num,point_num):
                x=global_path.poses[i+box].pose.position.x
                y=global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix=np.array(x_list)
            y_matrix=np.array(y_list)
            x_trans=x_matrix.T
            

            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            
            r=sqrt(abs(a*a+b*b-c))
            
          
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan


        
       

class purePursuit :
    '''
    경로 waypoint 위치와 현재 차량의 상태를 파악하여,
    경로 waypoint를 차량이 바라볼 수 있게 각도값과 속도를 조절함.
    '''
    def __init__(self):
        '''
        PurePursuit 계산에 필요한 초기값 설정
        '''
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.lfd=2
        self.min_lfd=2
        self.max_lfd=30
        self.vehicle_length=1
        self.steering=0
        
    def getPath(self,msg):
        '''
        경로 메세지 읽기
        '''
        self.path=msg  #nav_msgs/Path 
    
    
    def getEgoStatus(self,msg):
        '''
        현재 차량의 속도, x, y, z, yaw값 확인
        '''
        self.current_vel=msg.velocity  #kph
        self.vehicle_yaw=msg.yaw/180*pi   # rad
        self.current_postion.x=msg.position_x
        self.current_postion.y=msg.position_y
        self.current_postion.z=msg.position_z



    def steering_angle(self):
        '''
        경로 waypoint와 차량 위치를 이용하여 차량이 목표 waypoint를 바라보게 할 수 있는 회전값을 구함.
        거리값을 이용하여 lfd 값도 .
        '''
        vehicle_position=self.current_postion
        rotated_point=Point()
        self.is_look_forward_point= False

        

        for i in self.path.poses :
            path_point=i.pose.position
            dx= path_point.x - vehicle_position.x
            dy= path_point.y - vehicle_position.y
            rotated_point.x=cos(self.vehicle_yaw)*dx +sin(self.vehicle_yaw)*dy
            rotated_point.y=sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
 
            
            if rotated_point.x>0 :
                dis=sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                
                if dis>= self.lfd :
                    
                    self.lfd=self.current_vel/3.6
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    
                    break
        
        theta=atan2(rotated_point.y,rotated_point.x)

        if self.is_look_forward_point :
            self.steering=atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi * 0.6 #deg
            #print( self.steering)
            return self.steering 
        else : 
            #print("no found forward point")
            return 0
        

class cruiseControl:
    '''
    전역 및 지역으로 장애물을 판별한 후 거리에 따라 속도 
    '''
    def __init__(self,object_vel_gain,object_dis_gain):
        '''
        게인값 설정 및 물체 확인 초기 설정
        '''
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        self.object_vel_gain=object_vel_gain
        self.object_dis_gain=object_dis_gain


    def checkObject(self,ref_path,global_vaild_object,local_vaild_object,tl=[]):
        '''
        전역 물체의 [][0]번에 따라 0이면 사람, 3이면 표지판으로 판단한다.
        전역(global)으로 물체가 있으면 현 위치와 거리를 측정한다.
        거리가 3 미만이면 지역(local)으로 거리를 측정한다.
        '''
        self.object=[False,0]
        self.traffic=[False,0]
        self.Person=[False,0]
        if len(global_vaild_object) >0  :
            min_rel_distance=float('inf')
            for i in range(len(global_vaild_object)):
                for path in ref_path.poses :

                    if global_vaild_object[i][0]==0 :
                    
                        dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))

                        if dis<3:
                            
                            rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                            if rel_distance < min_rel_distance:
                                min_rel_distance=rel_distance
                                self.Person=[True,i]


                    if global_vaild_object[i][0]==3 :
                        traffic_sign='STOP'

                        if len(tl)!=0  and  global_vaild_object[i][3] == tl[0] :
                            if tl[1] == 48 or tl[1]==16   :   #
                                traffic_sign ='GO'
                        # print(traffic_sign)
                        if traffic_sign =='STOP':
                            dis=sqrt(pow(path.pose.position.x-global_vaild_object[i][1],2)+pow(path.pose.position.y-global_vaild_object[i][2],2))
                            
                            if dis<2.5 :
                                # @g0pher98 thankyou ^^ 
                                rel_distance= sqrt(pow(local_vaild_object[i][1],2)+pow(local_vaild_object[i][2],2))
                                if rel_distance < min_rel_distance:
                                    min_rel_distance=rel_distance
                                    self.traffic=[True,i]

    def acc(self,local_vaild_object,ego_vel,target_vel,status_msg):
        '''
        위의 함수에서 사람 혹은 표지판이 있다고 판별나면 속도 조절 시작
        시간 차, 기본 확보 공간을 설정하고 현 위치와 물체 간 거리 측정
        속도 게인과 거리 게인을 이용하여 추가 속도를 구함.
        목표 속도와 {현재 속도  + 추가 속도}를 비교하여 현재 속도를 조절.
        '''
        out_vel=target_vel
        pre_out_vel = out_vel

        if self.Person[0]==True:
            #print("ACC ON_person")
            Pedestrian=[local_vaild_object[self.Person[1]][1],local_vaild_object[self.Person[1]][2],local_vaild_object[self.Person[1]][3]]
            time_gap=0.6
            default_space=1
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(Pedestrian[0],2)+pow(Pedestrian[1],2))-3
            
            vel_rel=(Pedestrian[2]-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel


        if self.traffic[0] == True :
            print("Traffic_ON")   
            front_vehicle=[local_vaild_object[self.traffic[1]][1],local_vaild_object[self.traffic[1]][2],local_vaild_object[self.traffic[1]][3]]
            time_gap=0.3
            default_space=0.1
            dis_safe=ego_vel* time_gap+default_space
            dis_rel=sqrt(pow(front_vehicle[0],2)+pow(front_vehicle[1],2))-1
            
            vel_rel=(0-ego_vel)  
            
            v_gain=self.object_vel_gain
            x_errgain=self.object_dis_gain
            acceleration=vel_rel*v_gain - x_errgain*(dis_safe-dis_rel)    

            acc_based_vel=ego_vel+acceleration
            
            if acc_based_vel > target_vel : 
                acc_based_vel=target_vel
            
            if dis_safe-dis_rel >0 :
                out_vel=acc_based_vel
            else :
                if acc_based_vel<target_vel :
                    out_vel=acc_based_vel
            # print(dis_safe,dis_rel,- x_errgain*(dis_safe-dis_rel),out_vel)

            if dis_rel < 1 :
                out_vel = 0

        #print("out_vel", out_vel)
        return out_vel

class mgko_obj :
    '''
    물체(장애물) 설정
    '''
    def __init__(self):
        self.num_of_objects=0
        self.pose_x=[]
        self.pose_y=[]
        self.velocity=[]
        self.object_type=[]
        
        


class vaildObject :
    '''
    전역 및 지역으로 정지선, 장애물 정보 확인 후 
    '''
    def __init__(self,stop_line=[]):
        self.stop_line=stop_line
    def get_object(self,num_of_objects,object_type,pose_x,pose_y,velocity):
        '''
        설정된 장애물 정보 읽어오기
        '''
        self.all_object=mgko_obj()
        self.all_object.num_of_objects=num_of_objects
        self.all_object.object_type=object_type
        self.all_object.pose_x=pose_x
        self.all_object.pose_y=pose_y
        self.all_object.velocity=velocity


    def calc_vaild_obj(self,ego_pose):  # x, y, heading
        '''
        전역 경로 장애물 정보는 모든 설정된 장애물의 정보 포함
        지역 경로 장애물 정보는 차량의 현재 위치에 대한 상대적 위치(방향 포함)를 구하여 해당하는 장애물의 정보 포함.
        정지선이 설정된 경우 장애물과 동일하게 전역 정지선 및 지역 정지선 설정.
        '''
        global_object_info=[]
        loal_object_info=[]
        
        # if self.all_object.num_of_objects > 0:

        tmp_theta=ego_pose[2]
        tmp_translation=[ego_pose[0],ego_pose[1]]
        tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],
                        [sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],
                        [0,0,1]])
        tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],
                            [tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],
                            [0,0,1]])

        for num in range(self.all_object.num_of_objects):
            global_result=np.array([[self.all_object.pose_x[num]],[self.all_object.pose_y[num]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.append([self.all_object.object_type[num],self.all_object.pose_x[num],self.all_object.pose_y[num],self.all_object.velocity[num]])
                loal_object_info.append([self.all_object.object_type[num],local_result[0][0],local_result[1][0],self.all_object.velocity[num]])
        
        
        for line in self.stop_line:
            global_result=np.array([[line[0]],[line[1]],[1]])
            local_result=tmp_det_t.dot(global_result)
            if local_result[0][0]> 0 :
                global_object_info.append([3,line[0],line[1],line[2]])
                loal_object_info.append([3,local_result[0][0],local_result[1][0],line[2]])
        

        return global_object_info,loal_object_info

class pidController : ## 속도 제어를 위한 PID 적용 ##
    '''
    PD제어(+피드백제어)
    '''
    def __init__(self):
        '''
        PID 제어를 위한 초기 게인 설정 <- PID 최적화를 통해 게인값 조정이 기본
        '''
        self.p_gain=1.0
        self.i_gain=0.0
        self.d_gain=0.5
        self.controlTime=0.033
        self.prev_error=0
        self.i_control=0


    def pid(self,target_vel,current_vel):
        '''
        피드백 제어를 통해 p제어 및 d제어
        '''
        error= target_vel-current_vel
        
        p_control=self.p_gain*error
        self.i_control+=self.i_gain*error*self.controlTime
        d_control=self.d_gain*(error-self.prev_error)/self.controlTime

        output=p_control+self.i_control+d_control
        self.prev_error=error
        return output

########################  lattice  ########################

def latticePlanner(ref_path,global_vaild_object,vehicle_status,current_lane,
                    lattice_branch = 5,
                    lattice_gap = 0.3,
                    lattice_weight_gap = 10
                    ):
    '''
    %자세한 내용은 기술 보고서를 참조하는 것을 추천%
    레인(선)의 길이(look_distance), 개수(branch), 레인 간 거리(offset) 설정
    레인의 설정 값에 맞추어 레인 각각의 시작 및 끝 waypoint 설정
    가중치(weight)에 따라 weight가 가장 작은 레인 최종 선정
    전역 및 지역 경로를 동일한 방식으로 
    '''
    out_path=[]
    selected_lane=-1
    lattic_current_lane=current_lane
    look_distance=int(vehicle_status[3]*3.6*0.2*2)
    if look_distance < 2 :
        look_distance=2     #min 5m
    if look_distance > 3 :
        look_distance=3   
    if len(ref_path.poses)>look_distance :
        global_ref_start_point=(ref_path.poses[0].pose.position.x,ref_path.poses[0].pose.position.y)
        global_ref_start_next_point=(ref_path.poses[1].pose.position.x,ref_path.poses[1].pose.position.y)
        global_ref_end_point=(ref_path.poses[look_distance].pose.position.x,ref_path.poses[look_distance].pose.position.y)
        
        theta=atan2(global_ref_start_next_point[1]-global_ref_start_point[1],global_ref_start_next_point[0]-global_ref_start_point[0])
        translation=[global_ref_start_point[0],global_ref_start_point[1]]

        t=np.array([[cos(theta), -sin(theta),translation[0]],[sin(theta),cos(theta),translation[1]],[0,0,1]])
        det_t=np.array([[t[0][0],t[1][0],-(t[0][0]*translation[0]+t[1][0]*translation[1])   ],[t[0][1],t[1][1],-(t[0][1]*translation[0]+t[1][1]*translation[1])   ],[0,0,1]])



        world_end_point=np.array([[global_ref_end_point[0]],[global_ref_end_point[1]],[1]])
        local_end_point=det_t.dot(world_end_point)
        world_ego_vehicle_position=np.array([[vehicle_status[0]],[vehicle_status[1]],[1]])
        local_ego_vehicle_position=det_t.dot(world_ego_vehicle_position)
        
        lane_off_set = [(i - lattice_branch // 2) * lattice_gap for i in range(lattice_branch)]


        # print(lane_off_set)
        #lane_off_set=[3.9,2.6,1.3,0,-1.3,-2.6,-3.9]
        #lane_off_set=[2.6,2.5, 2.4, 2.3, 2.2, 2.1, 2.0, 1.9, 1.8, 1.7, 1.0, 0.9,0.8,0.7,0.6,0,-0.6,-0.7,-0.8,-0.9, -1.0, -1.7, -1.8, -1.9, -2.0, -2.1, -2.2, -2.3, -2.4, -2.5, -2.6  ] #Sangyoon
        
        local_lattice_points=[]
        for i in range(len(lane_off_set)):
            local_lattice_points.append([local_end_point[0][0],local_end_point[1][0]+lane_off_set[i],1])
            


        for end_point in local_lattice_points :
            lattice_path=Path()
            lattice_path.header.frame_id='map'
            x=[]
            y=[]
            x_interval=0.45   # default = 0.5
            xs=0
            xf=end_point[0]
            ps=local_ego_vehicle_position[1][0]

            pf=end_point[1]
            x_num=xf/x_interval

            for i in range(xs,int(x_num)) : 
                x.append(i*x_interval)
            
            a=[0.0,0.0,0.0,0.0]
            a[0]=ps
            a[1]=0
            a[2]=2.1*(pf-ps)/(xf*xf) # default = 3.0
            a[3]=-2.4*(pf-ps)/(xf*xf*xf) # default = -2.0

            for i in x:
                result=a[3]*i*i*i+a[2]*i*i+a[1]*i+a[0]
                y.append(result)


            for i in range(0,len(y)) :
                local_result=np.array([[x[i]],[y[i]],[1]])
                global_result=t.dot(local_result)

                read_pose=PoseStamped()
                read_pose.pose.position.x=global_result[0][0]
                read_pose.pose.position.y=global_result[1][0]
                read_pose.pose.position.z=0
                read_pose.pose.orientation.x=0
                read_pose.pose.orientation.y=0
                read_pose.pose.orientation.z=0
                read_pose.pose.orientation.w=1
                lattice_path.poses.append(read_pose)

            out_path.append(lattice_path)
        
        add_point_size=int(vehicle_status[3]*2*3.6)
        #print('add point',add_point_size)
        if add_point_size>len(ref_path.poses)-2:
            add_point_size=len(ref_path.poses)
        elif add_point_size<10 :
            add_point_size=10
        
        
         
        for i in range(look_distance,add_point_size):
            if i+1 < len(ref_path.poses):
                tmp_theta=atan2(ref_path.poses[i+1].pose.position.y-ref_path.poses[i].pose.position.y,ref_path.poses[i+1].pose.position.x-ref_path.poses[i].pose.position.x)
                
                tmp_translation=[ref_path.poses[i].pose.position.x,ref_path.poses[i].pose.position.y]
                tmp_t=np.array([[cos(tmp_theta), -sin(tmp_theta),tmp_translation[0]],[sin(tmp_theta),cos(tmp_theta),tmp_translation[1]],[0,0,1]])
                tmp_det_t=np.array([[tmp_t[0][0],tmp_t[1][0],-(tmp_t[0][0]*tmp_translation[0]+tmp_t[1][0]*tmp_translation[1])   ],[tmp_t[0][1],tmp_t[1][1],-(tmp_t[0][1]*tmp_translation[0]+tmp_t[1][1]*tmp_translation[1])   ],[0,0,1]])

                for lane_num in range(len(lane_off_set)) :
                    local_result=np.array([[0],[lane_off_set[lane_num]],[1]])
                    global_result=tmp_t.dot(local_result)

                    read_pose=PoseStamped()
                    read_pose.pose.position.x=global_result[0][0]
                    read_pose.pose.position.y=global_result[1][0]
                    read_pose.pose.position.z=0
                    read_pose.pose.orientation.x=0
                    read_pose.pose.orientation.y=0
                    read_pose.pose.orientation.z=0
                    read_pose.pose.orientation.w=1
                    out_path[lane_num].poses.append(read_pose)


        lane_weight = [abs((i - lattice_branch // 2) * lattice_weight_gap) for i in range(lattice_branch)]
        collision_bool = [False for _ in range(lattice_branch)]



        if len(global_vaild_object)>0:

            for obj in global_vaild_object :
                if  obj[0]==2 or obj[0]==1 : 
                    for path_num in range(len(out_path)) :
                        
                        for path_pos in out_path[path_num].poses :
                            
                            dis= sqrt(pow(obj[1]-path_pos.pose.position.x,2)+pow(obj[2]-path_pos.pose.position.y,2))
   
                            if dis<1.5: # default = 1.5
                                collision_bool[path_num]=True
                                lane_weight[path_num]=lane_weight[path_num]+100
                                break
        else :
            print("No Obstacle")
    
        # print(lane_weight)
        # print(collision_bool)

        selected_lane=lane_weight.index(min(lane_weight))
        #print(lane_weight,selected_lane)
        all_lane_collision=True
        
    else :
        #print("NO Reference Path")
        selected_lane=-1    

    return out_path,selected_lane

########################  lattice  #######################
