#!/usr/bin/env python

# Author: Connor McGuile
# Feel free to use in any way.

# A custom Dynamic Window Approach implementation for use with Turtlebot.
# Obstacles are registered by a front-mounted laser and stored in a set.
# If, for testing purposes or otherwise, you do not want the laser to be used,
# disable the laserscan subscriber and create your own obstacle set in main(),
# before beginning the loop. If you do not want obstacles, create an empty set.
# Implentation based off Fox et al.'s paper, The Dynamic Window Approach to 
# Collision Avoidance (1997).
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist, PointStamped,PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
import time

class Config():
    # simulation parameters

    def __init__(self):
        # robot parameter
        #NOTE good params:
        #NOTE 0.55,0.1,1.0,1.6,3.2,0.15,0.05,0.1,1.7,2.4,0.1,3.2,0.18
        self.max_speed = 0.5  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yawrate = 1.0  # [rad/s]
        self.max_accel = 2  # [m/ss]
        self.max_dyawrate = 2.5  # [rad/ss]
        self.v_reso = 0.05  # [m/s]
        self.yawrate_reso = 0.1  # [rad/s]
        self.dt = 0.5  # [s]
        self.predict_time = 3  # [s]
        self.to_goal_cost_gain = 20 #lower = detour
        self.speed_cost_gain = 4.5 #lower = faster
        self.obs_cost_gain = 2 #lower z= fearless
        self.robot_radius = 0.4  # [m]
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.goalX = 0.0
        self.goalY = 0.0
        self.goal_index = 1
        self.r = rospy.Rate(10)
        self.path = Path()
        self.tf = tf.TransformListener()
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.future_pathpub = rospy.Publisher('/course_agv/future_path',Path,queue_size=1)

    def updatepredicttime(self):
        if((math.sqrt((self.x - self.goalX)**2 + (self.y - self.goalY)**2))<0.5):
            self.predict_time=1.5
        else:
            self.predict_time=3

        
    # Callback for Odometry
    def assignOdomCoords(self, msg):
        # X- and Y- coords and pose of robot fed back into the robot config
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll,pitch,theta) = \
            euler_from_quaternion ([rot_q.x,rot_q.y,rot_q.z,rot_q.w])
        self.th = theta

    # Callback for attaining goal co-ordinates from Rviz Publish Point
    def getcurpos(self,msg):
        self.path = msg
        self.goalX = self.path.poses[self.goal_index].pose.position.x
        self.goalY = self.path.poses[self.goal_index].pose.position.y
        self.midpose_pub.publish(self.path.poses[self.goal_index])
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        #print("get_tf")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        (roll,pitch,theta) = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        #print("goal "+str(self.goalX)+" "+str(self.goalY))
        #print("robot "+str(self.x)+" "+str(self.y))
        self.th = theta

    def updategoal(self):
        if self.goal_index < len(self.path.poses)-1:
            
            self.goal_index=self.goal_index+2
            print("goalindex")
            print(len(self.path.poses)-1)
            print(self.goal_index)
            self.goalX = self.path.poses[self.goal_index].pose.position.x
            self.goalY = self.path.poses[self.goal_index].pose.position.y 
            self.midpose_pub.publish(self.path.poses[self.goal_index])
            return True           
        else:
            return False
        
    def updaterobotpos(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        #print("get_tf")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        (roll,pitch,theta) = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.th=theta


class Obstacles():
    def __init__(self):
        # Set of coordinates of obstacles in view
        self.obst = set()

    # Custom range implementation to loop over LaserScan degrees with
    # a step and include the final degree
    def myRange(self,start,end,step):
        i = start
        while i < end:
            yield i
            i += step
        yield end

    # Callback for LaserScan
    def assignObs(self, msg, config):
        #rospy.loginfo("assignobs")
        deg = len(msg.ranges)   # Number of degrees - varies in Sim vs real world
        self.obst = set()   # reset the obstacle set to only keep visible objects
        for angle in self.myRange(0,deg-1,6):
            distance = msg.ranges[angle]
            # only record obstacles that are within 4 metres away
            if (distance < 4):
                # angle of obstacle wrt robot(with regard to)
                # angle/2.844 is to normalise the 512 degrees in real world
                # for simulation in Gazebo, use angle/4.0
                # laser from 0 to 180
                scanTheta = (-180+angle*360/deg)*math.pi/180.0#(angle/2.844 + deg*(-180.0/deg)+90.0) *math.pi/180.0
                # angle of obstacle wrt global frame
                objTheta = config.th + scanTheta
                # back quadrant negative X negative Y
                if (objTheta < -math.pi):
                    # e.g -405 degrees >> 135 degrees
                    objTheta = objTheta + 2*math.pi
                # back quadrant negative X positve Y
                elif (objTheta > math.pi):
                    objTheta = objTheta - 2*math.pi
                # round coords to nearest 0.125m
                obsX = (config.x + (distance * math.cos(objTheta)))
                obsY = (config.y + (distance * math.sin(objTheta)))
                # determine direction of Y coord
                # if (objTheta < 0): # uncomment and comment line below for Gazebo simulation
                # add coords to set so as to only take unique obstacles
                self.obst.add((obsX,obsY))
                #print self.obst
        #rospy.loginfo("obs")
        #rospy.loginfo(self.obst)

# Model to determine the expected position of the robot after moving along trajectory
def motion(x, u, dt):
    # motion model
    # x = [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    if u[1]!=0:
        x[0] =x[0]+ (u[0]/u[1])*(math.sin(x[2]+u[1]*dt)-math.sin(x[2]))
        x[1] =x[1]+ (u[0]/u[1])*(math.cos(x[2])-math.cos(x[2]+u[1]*dt))
    else:
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    if(x[2]>math.pi):
        x[2]=x[2]-2*math.pi
    if(x[2]<-math.pi):
        x[2]=x[2]+2*math.pi
    x[3] = u[0]
    x[4] = u[1]

    return x

# Determine the dynamic window from robot configurations
def calc_dynamic_window(x, config):

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yawrate, config.max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_dyawrate * config.dt,
          x[4] + config.max_dyawrate * config.dt]

    #  [vmin, vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw   

# Calculate a trajectory sampled across a prediction time
def calc_trajectory(xinit, v, y, config):

    x = np.array(xinit)
    traj = np.array(x)  # many motion models stored per trajectory
    time = 0

    while time <= config.predict_time:
        # store each motion model along a trajectory
        x = motion(x, [v, y], config.dt)
        traj = np.vstack((traj, x))
        time += config.dt # next sample
    #print (traj)
    return traj

# Calculate trajectory, costings, and return velocities to apply to robot
def calc_final_input(x, u, dw, config, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    mydiction=dict()
    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_reso):
        for w in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = calc_trajectory(xinit, v, w, config)
            mydiction[(v,w)]=traj
            #rospy.loginfo(str(v)+" "+str(w)+" ")
            #rospy.loginfo(traj)
            # calc costs with weighted gains
            to_goal_cost = calc_to_goal_cost(traj, config) * config.to_goal_cost_gain
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])

            ob_cost = calc_obstacle_cost(traj, ob, config) * config.obs_cost_gain

            final_cost = to_goal_cost + speed_cost + ob_cost
            #rospy.loginfo(str(v)+" "+str(w)+" to_goal"+str(to_goal_cost)+" speed"+str(speed_cost)+" obs"+str(ob_cost))

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                rospy.loginfo(str(v)+" "+str(w)+" to_goal"+str(to_goal_cost)+" speed"+str(speed_cost)+" obs"+str(ob_cost))
                min_u = [v, w]

    print ("dic result")
    print(min_u)
    print(mydiction[(min_u[0],min_u[1])])            
    return min_u

# Calculate obstacle cost inf: collision, 0:free
def calc_obstacle_cost(traj, ob, config):
    skip_n = 1
    minr = float("inf")

    # Loop through every obstacle in set and calc Pythagorean distance
    # Use robot radius to determine if collision
    for ii in range(0, len(traj[:1])):
        for i in ob.copy():
            ox = i[0]
            oy = i[1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy
            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr

# Calculate goal cost via Pythagorean distance to robot
def calc_to_goal_cost(traj, config):
    # If-Statements to determine negative vs positive goal/trajectory position
    # traj[-1,0] is the last predicted X coord position on the trajectory
    mingoalc=10000
    #print(len(traj[:, 1]))
    dx = abs(config.goalX - traj[-1,0])
    dy = abs(config.goalY - traj[-1,1])
    cost = math.sqrt(dx**2 + dy**2)
    #rospy.loginfo(str(ii)+" "+str(cost))
            #rospy.loginfo(str(ii)+" "+str(cost))
    return cost

# Begin DWA calculations
def dwa_control(x, u, config, ob):
    # Dynamic Window control

    dw = calc_dynamic_window(x, config)        
    rospy.loginfo(dw)
    u = calc_final_input(x, u, dw, config, ob)
    i=0
    fupath = Path()
    fupath.header.seq = 0
    fupath.header.stamp = rospy.Time(0)
    fupath.header.frame_id = 'map'
    print("draw")
    pathx=config.x
    pathy=config.y
    print(pathx)        
    print(pathy)
    pathth=config.th
    print(pathth)
    while((i-1)<config.predict_time/config.dt):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        if (u[1]!=0):
            pathx =pathx +(u[0]/u[1])*(math.sin(x[2]+u[1]*config.dt)-math.sin(x[2]))
            pathy =pathy + (u[0]/u[1]*(math.cos(x[2])-math.cos(x[2]+u[1]*config.dt)))
        else:
            pathx =pathx + u[0] * math.cos(x[2]) * config.dt
            pathy =pathy + u[0] * math.sin(x[2]) * config.dt
        x[2]=x[2]+u[1]*config.dt
        if(x[2]>math.pi):
            x[2]=x[2]-2*math.pi
        if(x[2]<-math.pi):
            x[2]=x[2]+2*math.pi
        pose.pose.position.x = pathx
        pose.pose.position.y = pathy
        print(pose.pose.position.x)        
        print(pose.pose.position.y)
        
        pose.pose.position.z = 0.01
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        fupath.poses.append(pose)
        i=i+1  
    config.future_pathpub.publish(fupath)
    return u

# Determine whether the robot has reached its goal
def atGoal(config, x):
    # check at goal
    print(str(math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) ))
    if math.sqrt((x[0] - config.goalX)**2 + (x[1] - config.goalY)**2) \
        <= config.robot_radius:
        return True
    return False


def main():
    print(__file__ + " start!!")
    # robot specification
    config = Config()
    # position of obstacles
    obs = Obstacles()
    #subOdom = rospy.Subscriber("/odom", Odometry, config.assignOdomCoords)
    subLaser = rospy.Subscriber('/course_agv/laser/scan', LaserScan, obs.assignObs, config)
    #subGoal = rospy.Subscriber("/clicked_point", PointStamped, config.goalCB)
    #pub = rospy.Publisher("cmd_vel_mux/input/teleop", Twist, queue_size=1)
    vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
    path_sub = rospy.Subscriber('/course_agv/global_path',Path,config.getcurpos)
    speed = Twist()
    # initial state [x(m), y(m), theta(rad), v(m/s), omega(rad/s)]
    x = np.array([config.x, config.y, config.th, 0.0, 0.0])
    # initial linear and angular velocities
    u = np.array([0.0, 0.0])
    while not rospy.is_shutdown():
        if (atGoal(config,x) == False):
            u= dwa_control(x, u, config, obs.obst)
            #print ("my dict")
            #print(mydiction[u[0],u[1]])
            print("velocity")
            print(u)
            print("goal")
            print(config.goalX)
            print(config.goalY)
            config.updaterobotpos()
            config.updatepredicttime()
            x[0] = config.x
            x[1] = config.y
            x[2] = config.th
            x[3] = u[0]
            x[4] = u[1]
            speed.linear.x = x[3]
            speed.angular.z = x[4]
        else:
            # if at goal then stay there until new goal published
            updategoalcon=config.updategoal()
            config.updaterobotpos()
            if(updategoalcon==False):
                speed.linear.x = 0.0
                speed.angular.z = 0.0
        vel_pub.publish(speed)
        time.sleep(0.5)
    # runs until terminated externally
    


if __name__ == '__main__':
    rospy.init_node('mydwa')
    main()