#!/usr/bin/env python2
import rospy

from geometry_msgs.msg import PoseStamped

from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid,Path
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, WaypointList, AttitudeTarget
from geometry_msgs.msg import PoseStamped, TwistStamped,AccelStamped,PointStamped
from geometry_msgs.msg import Twist, Vector3Stamped, Pose, Point, Quaternion, Vector3,Accel
from visualization_msgs.msg import MarkerArray, Marker
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_msgs.msg import Header
import numpy as np
import tf
import math
import time
from scipy.optimize import minimize
from pid import PID

class Get_control:
    def __init__(self):
        self.pid=PID(6,0.2,4,-1.1,1.1)
        # rospy.init_node('get_control')
    
    def fun(self,x,trj_dt,px,py,pz,vx,vy,vz,pos_wp,lt_ctrl,pos_wp2):
        # speed =np.linalg.norm(np.array([vx,vy,vz]))
        # func = control_gain*np.linalg.norm(x[0:3])+speed_cost_gain*abs(min(trj_dt,np.sign(trj_dt)*50)) -np.linalg.norm(np.array([vx,vy,vz])+x[0:3]*trj_dt)+ 30*np.linalg.norm(((np.array([vx,vy,vz])+x[0:3]*trj_dt/2)*trj_dt) - para_g*loc_goal)
        traj_end = ((np.array([vx,vy,vz])+x[0:3]*trj_dt/2)*trj_dt)
        # if sum(traj_end*loc_goal)<0:
        #     para_d = 10
        # else:
        #     para_d = 1
        # traj_end_v = np.array([vx,vy,vz])+x[0:3]*2*trj_dt
        
        pp_1 = traj_end -pos_wp
        # pp_2 = traj_end
        # pp_3 = np.linalg.norm(pos_wp)
        # pp_l = np.linalg.norm(np.cross(pp_1,pp_2))/pp_3
        # pp_d = pp_l + 0.5*np.linalg.norm(pp_1)
        pp_d = np.linalg.norm(pp_1)
        # end_v = np.array([vx,vy,vz])+x[0:3]*trj_dt
        # end_v = end_v/np.linalg.norm(end_v)
        # traj_end2next = (pos_wp2-traj_end)
        # traj_end2next = traj_end2next/np.linalg.norm(traj_end2next)
        # cost_v2next = np.linalg.norm(traj_end2next-end_v)
        
        # if (pos_wp2!= pos_wp).any():
        #     pp_l2_v = (pos_wp2-pos_wp)/np.linalg.norm(pos_wp2-pos_wp) - traj_end_v/np.linalg.norm(traj_end_v)
        #     pp_l3 = np.linalg.norm(pp_l2_v)
        # else:
        #     pp_l3=0
            
        if (pos_wp2!= pos_wp).any() and np.linalg.norm([vx,vy,vz])>1.5:
            pp_l2 = 0
            for i in range(3,7):
                traj_end2 = ((np.array([vx,vy,vz])+x[0:3]/2*trj_dt*i/2)*2*trj_dt*i/2)
                pp_12 = traj_end2 -pos_wp2
                pp_22 = traj_end2-pos_wp
                pp_32 = np.linalg.norm(pos_wp2-pos_wp)
                pp_l2 += np.linalg.norm(np.cross(pp_12,pp_22))/pp_32
            # pp_12 = traj_end2 -pos_wp
            # pp_22 = traj_end2
            # pp_32 = np.linalg.norm(pos_wp)
            # pp_l2 = np.linalg.norm(np.cross(pp_12,pp_22))/pp_32
        else:
            pp_l2=0
        
        # pp_d2 = 0.7*np.linalg.norm(pp_1)+pp_l
        
        if lt_ctrl is None:
            lt_ctrl = x
        uav_speed = np.linalg.norm([vx,vy,vz])
        
        # if np.linalg.norm([vx,vy,vz]) > 8:
        #     speed_gain = 3-(np.linalg.norm([vx,vy,vz])-8)/(15-8)*3
        # else:
        #     speed_gain = 3
        if self.if_acc: #uav_speed < 1 and 
            speed_gain = 6
        else:
            speed_gain = 3
        # if uav_speed>self.switch_vel and self.switch_vel+0.1>uav_speed:
        #     control_gain = 0
        # else:
        control_gain= max(0.1, 0.8 - 0.1*uav_speed)
        func = control_gain*np.linalg.norm(x[0:3]-lt_ctrl)-speed_gain*np.linalg.norm(np.array([vx,vy,vz])+x[0:3]*trj_dt) + 50*pp_d+ 10*pp_l2 # +10*pp_l3
        return func   
    def get_angle(self,a,b):
        aa=np.linalg.norm(a)
        bb=np.linalg.norm(b)
        cc=np.linalg.norm(a-b)
        return math.acos((aa**2+bb**2-cc**2)/(2*aa*bb))
    def get_vel(self,set_points,local_pos,ct_goal,waypoints_cv,slen,t2,time_pred,uav_vel,mn_speed,lc_speed_list,
                speed_range,m_speed,max_accel,pred_coe,lt_ctrl,y,turn_points,yaw_gnum,accel_d,decel_d,
                decel_rad,nst_tn_pt_last,opt_time,low_t,last_if_slow):
        # max_speed = 2
        self.switch_vel = 0.5
        self.set_vel = 1.2
        turn_points = turn_points[1::]
        self.if_slow = 0
        self.if_acc = 0
        if nst_tn_pt_last > len(turn_points)-2:
            nst_tn_pt = nst_tn_pt_last
            print("nst_tn_pt",nst_tn_pt)
        elif nst_tn_pt_last !=0:
            nst_tn_pt = np.argmin(np.linalg.norm(turn_points[nst_tn_pt_last:nst_tn_pt_last+2]-local_pos,axis=1)) + nst_tn_pt_last
            if (turn_points[nst_tn_pt] == turn_points[min(nst_tn_pt+1,len(turn_points)-1)]).all() or np.linalg.norm(turn_points[nst_tn_pt] - turn_points[min(nst_tn_pt+1,len(turn_points)-1)])<0.5:
                nst_tn_pt+=1
        else:
            nst_tn_pt = np.argmin(np.linalg.norm(turn_points[0:2]-local_pos,axis=1))
        # print("turn points:",len(turn_points),nst_tn_pt,turn_points[nst_tn_pt:nst_tn_pt+2])
        # print('speed list lenth',len(lc_speed_list),lc_speed_list[nst_tn_pt])
        max_speed = lc_speed_list[min(nst_tn_pt,len(lc_speed_list)-1)]
        # print("nst_tn_pt,lc_speed_list",nst_tn_pt,lc_speed_list,turn_points)
        uav_speed = np.linalg.norm(uav_vel)
        # if uav_speed>m_speed:
        #     max_speed = m_speed
        if m_speed>max_speed:   
            decel_mt = abs(uav_speed-max_speed)/(0.4*max_accel)
            decel_rad = max(decel_rad,uav_speed*decel_mt-0.5*(0.4*max_accel)*decel_mt**2)
        print("deccel radius!",decel_rad,max_speed,m_speed,uav_speed)
        wpnxt = waypoints_cv[min(len(waypoints_cv)-1,ct_goal+1)]-local_pos
        if_turned = np.linalg.norm(uav_vel/uav_speed - wpnxt/np.linalg.norm(wpnxt))
        if (time.time()-low_t<=2 or (np.linalg.norm(turn_points[nst_tn_pt:min(nst_tn_pt+3,len(turn_points)-1)]-local_pos,axis=1)<decel_rad).any()):
            if m_speed>=max_speed:
                m_speed = min(m_speed,uav_speed)-decel_d
                print("slow down1!",max_speed,uav_speed)
                self.if_slow = 1
                low_t = time.time()
            elif uav_speed>=m_speed and (last_if_slow==1 or last_if_slow==2):#and uav_speed>self.set_vel:
                print("slow down2!",max_speed,uav_speed)
                self.if_slow = 2
                low_t = time.time()
            elif max_speed == speed_range[2] and (uav_speed < self.switch_vel or if_turned < 0.2): # and time.time()-low_t>2:
                print("start to fly straight,speed up!")
                m_speed = min(m_speed+accel_d,max_speed)
                self.if_acc = 1
            else:
                print("slow down3!",max_speed,uav_speed)
                self.if_slow = 3
        else:
            print("not in slow area, speed up")
            self.if_acc = 2
            if m_speed< speed_range[2]:
                m_speed = min(m_speed+accel_d,speed_range[2])
        # if if_stt==0 and self.if_slow==0:
        #     m_speed = np.clip(m_speed,m_speed,uav_speed+0.5*decel_d)
        # else:
        m_speed = np.clip(m_speed,0,uav_speed+2.5)
        # if abs(m_speed-uav_speed)>decel_d:
            
        #     m_speed = uav_speed + np.sign(m_speed-uav_speed)*decel_d
        # max_speed = m_speed
        t1=time.time()
        
        if ct_goal+1<len(waypoints_cv) :
            bb= np.linalg.norm(waypoints_cv[ct_goal]-waypoints_cv[ct_goal+1])
            # while ct_goal+1<len(waypoints_cv) and bb <slen*0.8:

            aa= np.linalg.norm(waypoints_cv[ct_goal]-local_pos)
            
            cc = np.linalg.norm(waypoints_cv[ct_goal+1]-local_pos) 
        if ct_goal+1<len(waypoints_cv):
            if_turn = (abs(self.get_angle(uav_vel,waypoints_cv[ct_goal+1]-local_pos))<math.pi/5 or self.if_slow==0 or self.if_slow==3 or uav_speed>speed_range[0]*1.2)
        else:
            if_turn = 1
        if ct_goal+1<len(waypoints_cv) and if_turn and \
            ((aa**2+bb**2-cc**2)/(2*aa*bb)>-0.7 or np.linalg.norm(waypoints_cv[ct_goal]-local_pos)<0.7*slen or t1-t2>0.4*time_pred):  #
          
            t2=time.time()
            
            
            # time_pred = min(0.6*slen/np.linalg.norm(uav_vel),time_pred)
            print("goal update!",time_pred)
            if (bb <slen*0.15 or opt_time*uav_speed>slen*0.3 or (self.if_slow==0 and np.linalg.norm(waypoints_cv[ct_goal+1]-local_pos)<1.6*slen)):# and ct_goal+2<len(waypoints_cv):
                ct_goal +=1
                # bb= np.linalg.norm(waypoints_cv[ct_goal]-waypoints_cv[ct_goal+1])
            ct_goal+=1
            time_pred = np.linalg.norm(waypoints_cv[min(ct_goal+1,len(waypoints_cv)-1)]-local_pos)/max(uav_speed,mn_speed)
        # print("ct_goal,len(waypoints_cv",ct_goal,len(waypoints_cv))
        
        if ct_goal+1<len(waypoints_cv):
            # pos_wp = (waypoints_cv[ct_goal]+waypoints_cv[ct_goal+1])/2
            pos_wp =  waypoints_cv[ct_goal]-local_pos
            # if np.linalg.norm(pos_wp) < slen:
            pos_wp=slen/np.linalg.norm(pos_wp)*(pos_wp)
            pos_wp2 = waypoints_cv[ct_goal+1]-local_pos
            if np.linalg.norm(pos_wp2-pos_wp)<0.5*slen:
                pos_wp2 =0.5*slen*(pos_wp2-pos_wp)/np.linalg.norm(pos_wp2-pos_wp)+pos_wp
            # if (waypoints_cv[ct_goal]-local_pos)[2]>0:
            #     pos_wp[2] = (waypoints_cv[ct_goal]-local_pos)[2]
        else:
            pos_wp = set_points[-1]-local_pos 
            pos_wp = slen/np.linalg.norm(pos_wp)*(pos_wp)
            pos_wp2 = pos_wp
        if ct_goal+yaw_gnum<len(waypoints_cv):
            yaw_wp = waypoints_cv[ct_goal+yaw_gnum]-local_pos
        else:
            yaw_wp = set_points[-1]-local_pos 

        

        vx,vy,vz = uav_vel
        px,py,pz = local_pos
        x0=(uav_speed)*(pos_wp/np.linalg.norm(pos_wp))
        pred_trj = []
        if uav_speed < self.switch_vel or (m_speed<self.set_vel and self.if_slow==3):
            vel = pos_wp2/np.linalg.norm(pos_wp2) *self.set_vel
            trj_dt = np.linalg.norm(pos_wp)/(max(0,uav_speed*self.set_vel))
            print("the velocity to goal!")
        elif (self.if_acc==2 and self.switch_vel<uav_speed < self.set_vel-0.2):
            vel = pos_wp2/np.linalg.norm(pos_wp2) *(self.set_vel)
            m_speed = self.set_vel
            # trj_dt = np.linalg.norm(pos_wp)/(max(0,uav_speed*self.set_vel))
            print("start,the velocity to goal!")
        else:
            t_b = time.time()
            bons = ((-max_accel,max_accel), (-max_accel, max_accel), (-0.5*max_accel, max_accel))
         
            trj_dt = np.linalg.norm(pos_wp)/(max(0,uav_speed*1.4))
            # trj_dt = np.linalg.norm(pos_wp)/m_speed
            cons = (#{'type': 'ineq', 'fun': lambda x: np.linalg.norm([vx+x[0]*trj_dt,vy+x[1]*trj_dt,vz+x[2]*trj_dt])+mn_speed},
                    {'type': 'ineq', 'fun': lambda x: -np.linalg.norm([vx+x[0]*trj_dt,vy+x[1]*trj_dt,vz+x[2]*trj_dt])+m_speed,
                      'jac' : lambda x: np.array([2*trj_dt*(vx+x[0]*trj_dt),2*trj_dt*(vy+x[1]*trj_dt),2*trj_dt*(vz+x[2]*trj_dt)])/(-2*np.linalg.norm([vx+x[0]*trj_dt,vy+x[1]*trj_dt,vz+x[2]*trj_dt]))}
                  )
            res = minimize(self.fun, x0, args=(trj_dt,px,py,pz,vx,vy,vz,pos_wp,lt_ctrl,pos_wp2),method='SLSQP',\
                        options={'maxiter':10},constraints=cons,bounds = bons, tol = 1e-2)
            traj_end = ((np.array([vx,vy,vz])+res.x[0:3]*trj_dt/2)*trj_dt)
            pp_1 = traj_end -pos_wp
            pp_2 = traj_end
            pp_3 = np.linalg.norm(pos_wp)
            pp_l = np.linalg.norm(np.cross(pp_1,pp_2))/pp_3
            pp_d=min(np.linalg.norm(traj_end -pos_wp),pp_l)
            opt_time = time.time()-t_b
            print("static traj end difference",pp_d,res.status,res.success,res.nit,"time cost opt:",opt_time)
            lt_ctrl=res.x
              # max(np.sign(res.x[2]),0)*
            # if vz+res.x[2]*trj_dt > max_accel and trj_dt > 0:
            #     res.x[2] = (3 - vz)/trj_dt
            # elif vz+res.x[2]*trj_dt < -2 and trj_dt > 0:
            #     res.x[2] = (-2 - vz)/trj_dt
            # elif np.linalg.norm(np.array([vx,vy,vz])+res.x[0:3]*trj_dt) > self.m_speed:
            #     res.x[0:3] = res.x[0:3]/np.linalg.norm(res.x[0:3])*(self.m_speed - np.linalg.norm(np.array([vx,vy,vz])))/trj_dt
            # elif np.linalg.norm(res.x[0:3]) > self.max_accel:
            #     res.x[0:3] = res.x[0:3]/np.linalg.norm(res.x[0:3])*self.max_accel
            # if pz-downb<0 and trj_dt >0:
            #     res.x[2] = 0.5*max_accel
            # if pz-upb>-1 and trj_dt >0:
            #     res.x[2] = -0.7
            # if self.self.if_slow :
            #     pred_coe*=1.5
            # res.x[2] = max(np.linalg.norm(res.x[0:2])/(max_accel*1.414),0.1)*0.56+res.x[2]
            
            # if np.linalg.norm(res.x[0:2])>1.6:
            #     res.x[0:2] = 1.6*res.x[0:2]/np.linalg.norm(res.x[0:2])
            
            # if res.x[2]<0:
            #     res.x *= 0.1
            if self.if_acc:
                # acc_mag = np.linalg.norm(res.x*trj_dt)
                # acc_uni = res.x*trj_dt/acc_mag
                # acc_mag = max(1,np.linalg.norm(res.x*trj_dt))
                # print("acc_mag",acc_mag,trj_dt)
                # vel=uav_vel+acc_uni*acc_mag
                # if max(abs(uav_vel))>10:
                #     res.x[np.argmax(abs(uav_vel))] *= 1.2
                #     # res.x[2] *= 0.5
                print("traj_dt",trj_dt,res.x*trj_dt,"if_acc,if_slow",self.if_acc,self.if_slow)
                vel=uav_vel+res.x*trj_dt
            else:
                vel=uav_vel+res.x*trj_dt*pred_coe
            if (self.if_acc==1 and np.linalg.norm(vel) < self.set_vel) or self.if_slow==3: # or (m_speed<self.set_vel and self.if_slow==2)\
               # or ():
                vel = max(self.set_vel,m_speed)*vel/np.linalg.norm(vel)
            for ti in range(11):
                tti = trj_dt*(ti/5)
                pred_trj.append((np.array([vx,vy,vz])+res.x[0:3]*tti/2)*tti+local_pos)
            print("control,vel",res.x,vel,uav_vel,m_speed) 
        if self.if_slow==1 or self.if_slow==2:
            vel[0:2] *= 0.85
        
        # elif self.if_acc:
        #     vel /= pred_coe
        # if vel[2] < -0.1:
        #     vel[2] = 0.1

        
        yaw=math.atan2(yaw_wp[1],yaw_wp[0])
        # y = controller.eular[2]
        if abs(yaw-y)>math.pi:
            yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
          
            # yaw=math.atan2(pos_wp[1],pos_wp[0])
            # if abs(yaw-y)>math.pi:
            #     yaw=(2*math.pi-abs(yaw))*np.sign(-yaw)
        dyaw=self.pid.step(yaw-y)
        return vel,dyaw,yaw,pred_trj,ct_goal,time_pred,t2,m_speed,lt_ctrl,pos_wp,pos_wp2,nst_tn_pt,self.if_slow,opt_time,low_t
    def get_curve(self,set_points,rds,slen,eta,speed_range,spd_angle_shrd):
        bi=1
        min_angle = spd_angle_shrd[0]
        max_angle = spd_angle_shrd[1]
        
        max_speed = speed_range[1]
        min_speed = speed_range[0]
        use_points = np.array([set_points[0].copy()])
        dc=[]
        max_v = [] #[speed_range[2]]
        ang_list=[]
        i=1
        while i < len(set_points):
            aa = np.linalg.norm(set_points[i-1]-set_points[i])
            if aa < 0.5:
                set_points = np.delete(set_points,i,axis=0)
            else:
                i +=1
        i=1
        for i in range(1,len(set_points)-1):
            aa = np.linalg.norm(set_points[i-1]-set_points[i])
            bb = np.linalg.norm(set_points[i+1]-set_points[i])
            cc = np.linalg.norm(set_points[i+1]-set_points[i-1])
            angcn = max(0.01,math.acos((aa**2 + bb**2-cc**2)/(2*aa*bb)))
            # angc.append(angcn)
            
            dturn = min(5,rds/math.tan(abs(angcn)/2))
            # if abs(angcn) > spd_angle_shrd[2]:
            ang_list.append(angcn)
            # if aa<rds or bb<rds:
            #     max_v += [*min_speed)]*2
            if abs(angcn) > max_angle:
                max_v += [speed_range[2]]*2
            elif abs(angcn) < min_angle:
                max_v += [min_speed]*2
            else:
                max_v += [min_speed+(max_speed-min_speed)*((abs(angcn)-min_angle)/(max_angle-min_angle))**1.6]*2
            if aa<rds or bb<rds:
                max_v[-2::] = list(np.array(max_v[-2::])*min(aa,bb)/rds)
            max_v.append(speed_range[2])
            if i ==1:
                dc.append(dturn)
            dc.append(dturn)
            dc.append(dturn)
            
        print("angles:",ang_list,len(dc))
        dc.append(dturn)
        max_v += [max_speed*0.8]
        max_v += [max_speed*0.5]
        for j in range(len(set_points)-1):
            if np.linalg.norm(set_points[j+1]-set_points[j]) < 2*rds:
                rds_t = np.linalg.norm(set_points[j+1]-set_points[j])/2
            else:
                rds_t = rds
            # if j+2 < len(set_points):
                
            # use_points=np.r_[use_points,[(set_points[j]*(2-beta)+set_points[j+1]*beta)/2]]
            # use_points=np.r_[use_points,[(set_points[j]*beta+set_points[j+1]*(2-beta))/2]]       
            # use_points=np.r_[use_points,[set_points[j+1]]]
            v_init = (set_points[j+1]-set_points[j])/np.linalg.norm(set_points[j+1]-set_points[j])
            if np.linalg.norm(set_points[j+1]-set_points[j]) < dc[2*j] + dc[2*j+1]:
                dc[2*j] = np.linalg.norm(set_points[j+1]-set_points[j])/2
                dc[2*j+1] = dc[2*j]
            # else:
            #     rds_t = rds
            use_points=np.r_[use_points,[set_points[j]+v_init*dc[2*j]]]
            use_points=np.r_[use_points,[set_points[j+1]-v_init*dc[2*j+1]]]       
            use_points=np.r_[use_points,[set_points[j+1]]]
        use_points = np.delete(use_points,1,axis=0)
        if len(use_points)<4:
            b_points = use_points[bi:bi+3]
            total_lth=0
            for i in range(len(b_points)-1):
                total_lth=total_lth+np.linalg.norm(b_points[i+1]-b_points[i])
            tt=np.arange(0,1,slen/total_lth)
            waypoints_cv = []
            for ti in tt:
                waypoints_cv.append((1-ti)**2*b_points[0] + eta*ti*(1-ti)*b_points[1]+ti**2*b_points[2])  # B-spline
            waypoints_cv=waypoints_cv[1::]
        else:
            waypoints_cv1 = [use_points[bi]]
            flag=1
            while bi+3<=len(use_points):
                if flag ==1:
                    flag = 0
                    total_lth = np.linalg.norm(use_points[bi]-use_points[bi-1])
                    
                    tt=np.arange(0,1,slen/total_lth)
                    for ti in tt:
                        waypoints_cv1.append((1-ti)*use_points[bi-1] + (ti)*use_points[bi])        
                else:
                    b_points = use_points[bi:bi+3]
                    total_lth=0
                    for i in range(len(b_points)-1):
                        total_lth=total_lth+np.linalg.norm(b_points[i+1]-b_points[i])
    
                        
                    # if bi ==0:
                        
                    #     tt=np.arange(slen/total_lth,1,slen/total_lth)
                    # else:
                    tt=np.arange(0,1,slen/total_lth)
                    
                    print("b_points",b_points,"total_len:",total_lth)
                    for ti in tt:
                        waypoints_cv1.append((1-ti)**2*b_points[0] + eta*ti*(1-ti)*b_points[1]+ti**2*b_points[2])
                        
                    # waypoints_cv = np.r_[np.array(waypoints_cv[0:-1]),np.array(waypoints_cv1)]
                    # if bi ==0:
                    #     bi+=2
                    # else:
                    bi=bi+3
                    flag = 1
            b_points = use_points[bi-1:bi+2]
            total_lth=0
            for i in range(len(b_points)-1):
                total_lth=total_lth+np.linalg.norm(b_points[i+1]-b_points[i])
            tt=np.arange(0,1,slen/total_lth)
            for ti in tt:
                waypoints_cv1.append((1-ti)**2*b_points[0] + eta*ti*(1-ti)*b_points[1]+ti**2*b_points[2])
            waypoints_cv = np.array(waypoints_cv1)
        return waypoints_cv,use_points,max_v
        