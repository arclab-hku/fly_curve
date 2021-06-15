#!/usr/bin/env python2
import rospy

from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Path
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker,MarkerArray
import numpy as np
import tf
from get_control import Get_control
import math
import time
class UAVController(object):
    def register(self):
        self.ros_data = {}
        self.ros_data["local_position"]=None
        self.vel = None
        rospy.init_node('UAV_controller')
        rospy.loginfo("UAV_controller started!")
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.pos_setvel_pub = rospy.Publisher(
            'mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/uav_path', Path, queue_size=10)
        self.stpath_pub = rospy.Publisher('/st_path', Path, queue_size=10)
        self.cvpath_pub = rospy.Publisher('/cv_path', Path, queue_size=10)
        self.predtrj_pub = rospy.Publisher('/pred_path', Path, queue_size=10)
        self.state_sub = rospy.Subscriber('mavros/state',
                                          State,
                                          self.state_callback)
        self.locwp_publisher = rospy.Publisher("local_wp", MarkerArray, queue_size=1)
        self.bp_publisher = rospy.Publisher("bezier_points", MarkerArray, queue_size=1)
        self.vel_sub=rospy.Subscriber('mavros/local_position/velocity_local',TwistStamped,self.vel_callback)
    
    def vel_callback(self,vel):
        self.vel = np.array([vel.twist.linear.x,vel.twist.linear.y,vel.twist.linear.z])
    def publish_bp(self,pts,t_points,p_index):
        bpts = MarkerArray()
        len_pts = len(pts)
        pts = np.r_[np.array(pts),np.array(t_points),np.array([t_points[min(p_index,len(t_points)-1)]])]
        for j in range(len(pts)):
            point=pts[j]
            bp = Marker()
            bp.header.frame_id = "map"
            bp.id = j
            bp.header.stamp = rospy.Time.now()
            bp.type = Marker.SPHERE
            bp.pose.position.x = point[0]
            bp.pose.position.y = point[1]
            bp.pose.position.z = point[2]
            bp.pose.orientation.x = 0.0
            bp.pose.orientation.y = 0.0
            bp.pose.orientation.z = 0.0
            bp.pose.orientation.w = 1.0
            bp.scale.x = 0.2
            bp.scale.y = 0.2
            bp.scale.z = 0.2
            if j==len(pts)-1:
                bp.scale.x = 0.4
                bp.scale.y = 0.4
                bp.scale.z = 0.4
                bp.color.a = 1
                bp.color.r = 0.9
                bp.color.g = 0.1
                bp.color.b = 0.9    
            elif j > len_pts-1:
                bp.scale.x = 0.4
                bp.scale.y = 0.4
                bp.scale.z = 0.4
                bp.color.a = 1
                bp.color.r = 0.6
                bp.color.g = 1.0
                bp.color.b = 0.0
            else:
                bp.color.a = 1
                bp.color.r = 0.6
                bp.color.g = 0.6
                bp.color.b = 0.0   

            bpts.markers.append(bp)
        self.bp_publisher.publish(bpts)
    def publish_locwp(self,wp,wp2,if_slow):
        wps = MarkerArray()
        i=0
        for wpi in [wp,wp2]:
            locwp = Marker()
            locwp.header.frame_id = "map"
            locwp.id = i
            locwp.header.stamp = rospy.Time.now()
            locwp.type = Marker.SPHERE
            locwp.pose.position.x = wpi[0]
            locwp.pose.position.y = wpi[1]
            locwp.pose.position.z = wpi[2]
            locwp.pose.orientation.x = 0.0
            locwp.pose.orientation.y = 0.0
            locwp.pose.orientation.z = 0.0
            locwp.pose.orientation.w = 1.0
            locwp.scale.x = 0.25
            locwp.scale.y = 0.25
            locwp.scale.z = 0.25
            if if_slow:
                locwp.scale.x = 0.35
                locwp.scale.y = 0.35
                locwp.scale.z = 0.35
          
            if i == 1:
                if if_slow:
                    locwp.color.a = 1
                    locwp.color.r = 1.0
                    locwp.color.g = 0
                    locwp.color.b = 0
                else:
                    locwp.color.a = 1
                    locwp.color.r = 0
                    locwp.color.g = 0
                    locwp.color.b = 1.0   
            else:
                locwp.color.a = 1
                locwp.color.r = 0
                locwp.color.g = 1
                locwp.color.b = 0.3  
            i+=1
            wps.markers.append(locwp)
       

        self.locwp_publisher.publish(wps)  
    def local_position_callback(self, data):
        self.ros_data["local_position"] = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z])
        qx=data.pose.orientation.x
        qy=data.pose.orientation.y
        qz=data.pose.orientation.z
        qw=data.pose.orientation.w
#        self.path.append([rx,ry,rz])

        rpy_eular=tf.transformations.euler_from_quaternion((qx,qy,qz,qw))
        self.eular = np.array(rpy_eular)
      # print(controller.ros_data["local_position"])
    def set_local_position(self,x,y,z,yaw=0):
        pos = PoseStamped()
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
        quaternion = quaternion_from_euler(0, 0, yaw)
        pos.pose.orientation = Quaternion(*quaternion)
        pos.header = Header()
        pos.header.frame_id = "map"
        pos.header.stamp = rospy.Time.now()
        self.pos_setpoint_pub.publish(pos)
        
    def state_callback(self, data):
        self.state = data
    def set_vel(self,vx,vy,vz,dyaw):
        vel = TwistStamped()
        vel.header = Header()
        vel.header.frame_id = "map"
        vel.header.stamp = rospy.Time.now()
        vel.twist.linear.x = vx
        vel.twist.linear.y = vy
        vel.twist.linear.z = vz
        vel.twist.angular.x = 0#pid.step(0-r)
        vel.twist.angular.y = 0#pid.step(0-p)\

        vel.twist.angular.z = dyaw
        # print("dyaw,omega_y",dyaw,vel.twist.angular.z)
        # vel.twist.angular.z = 0
        self.pos_setvel_pub.publish(vel)
    def publish_path(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.path_pub.publish(path)
    def publish_predtrj(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)

        self.predtrj_pub.publish(path)
    def publish_stpath(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.stpath_pub.publish(path)
    def publish_cvpath(self, data):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for d in data:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = d[0]
            pose.pose.position.y = d[1]
            pose.pose.position.z = d[2]

            path.poses.append(pose)
        self.cvpath_pub.publish(path)
if __name__ == '__main__':
    controller = UAVController()
    controller.register()
    controller.state=None
    Get_control = Get_control()
    # beta = 0.5 # <1, the smaller, the curve is closer to the straight path
    
    while controller.ros_data["local_position"] is None or controller.vel is None:
        
        rospy.sleep(0.05)
    
    local_pos = controller.ros_data["local_position"]
    # set_points = np.array([local_pos,[20,20,4],[16,4,3],[-20,-18,2],[-18,6,3],[0,0,2]]) # the pre-assigned waypoints
    # set_points = np.array([local_pos,[50,0,4],[50,4,3],[0,4,2],[0,8,3],[50,8,2],[50,12,4],[0,12,4],[0,0,4]]) 
    # set_points = np.array([local_pos,[25,0,4],[50,0,4],[50,25,3],[0,25,2],[50,0,3],[50,50,2],[50,0,4],[0,0,4]])
    set_points = np.array([local_pos,[25,2,4],[50,0,4],[50,25,3],[-100,25,2],[50,0,3],[50,50,2],[50,0,4],[0,0,4]])
    # set_points = np.array([local_pos,[100,0,4],[100,1,4],[0,1,3],[0,2,2],[100,2,3],[100,3,2],[0,4,4],[0,0,4]])
    speed_range = [1.0,6,15]
    # set_points = np.array([local_pos,[25,0,4],[50,0,4],[0,0,4]])
    # set_points[:,0:2] = set_points[:,0:2]*2
    mn_speed = 0.3 #not the constrain
    max_accel = 6
    # speed_range = [1.5,6,9]
    spd_angle_shrd = [math.pi/6,math.pi*9/10] #,math.pi*5/6
    accel_d = 0.1
    decel_d = 0.2
    m_speed = 1 # start the flight from a lower speed
    slen=0.9
    rds = 3.5
    eta = 2
    pred_coe = 0.7
    yaw_gnum = 6
    decel_rad = 3
    ct_goal = 0
    path=[]
    lt_ctrl=None
    time_pred=0
    t2=float('Inf')
    pred_trj=[]
    y=0
    rais=0
    nst_tn_pt=0
    opt_time = 0
    low_t = 0
    set_points[0,2] = set_points[1,2] 
    waypoints_cv,turn_points,lc_max_speed = Get_control.get_curve(set_points,rds,slen,eta,speed_range,spd_angle_shrd)
    # print("turn_points,lc_max_speed",turn_points,lc_max_speed)
    
    while not rospy.is_shutdown():
        # break
        pos_wp= np.zeros(3)
        pos_wp2= np.zeros(3)
        local_pos = controller.ros_data["local_position"]
        uav_vel = controller.vel
        print("current pose:",local_pos,rais)
        if controller.state is not None and controller.state.connected:
            if not (controller.state.mode == "OFFBOARD" and controller.state.armed):
                for kk in range(10):
                    controller.set_local_position(0,0,1)
                print("ifoffboard:",controller.state.mode)
                continue
        if rais ==0:
            while np.linalg.norm(local_pos-set_points[0])>0.3:
                controller.set_local_position(set_points[0,0],set_points[0,1],set_points[0,2]+0.2)
                local_pos = controller.ros_data["local_position"]
            rais+=1
        t1 = time.time()
        if np.linalg.norm(set_points[-1]-local_pos)<2 and nst_tn_pt>len(turn_points)-2:
            for ii in range(100):
                controller.set_vel(0,0,0,0)
            for iii in range(100):
                controller.set_local_position(set_points[-1,0],set_points[-1,1],set_points[-1,2],y)
            print("goal reached !!")
        else:
            y = controller.eular[2]
            vel,dyaw,yaw,pred_trj,ct_goal,time_pred,t2,m_speed,lt_ctrl,pos_wp,pos_wp2,nst_tn_pt,if_slow,opt_time,low_t = Get_control.get_vel(set_points,
                                              local_pos,ct_goal,waypoints_cv,slen,t2,time_pred,uav_vel,mn_speed,
                                              lc_max_speed,speed_range,m_speed,max_accel,pred_coe,lt_ctrl,y,turn_points,
                                              yaw_gnum,accel_d,decel_d,decel_rad,nst_tn_pt,opt_time,low_t)
            # vel = np.array([0.1,0.1,0.1])
            controller.set_vel(vel[0],vel[1],vel[2],dyaw)
            print('set_vel,ct_goal,waypoints_cv[ct_goal]',vel,ct_goal,waypoints_cv[np.clip(ct_goal,0,len(waypoints_cv)-1)])
        
        path.append(local_pos)
        if len(path)>200:
            del path[int(np.random.rand(1)*200)]
        print("time cost:",time.time()-t1)
        controller.publish_path(path)
        controller.publish_stpath(set_points)
        # controller.publish_cvpath(waypoints_cv)
        controller.publish_bp(waypoints_cv,turn_points,nst_tn_pt+1)
        controller.publish_locwp(pos_wp+local_pos,pos_wp2+local_pos,if_slow)
        controller.publish_predtrj(pred_trj)
                
        rospy.sleep(0.02)