#!/usr/bin/env python
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
import sys
import time
import numpy as np
import numpy as np
global mycmap
mycmap=[]
def mypictrans2(x, y):
    picpos =int(129 * y + x)
    return picpos

def mypictrans(x, y):
    picpos =int(43 * y + x)
    return picpos

def callbackgoal(data):
    global goalx
    goalx=data.pose.position.x
    global goaly
    goaly=data.pose.position.y
    rospy.loginfo("goalx="+str(goalx)+"goaly="+str(goaly) )
    global a_star
    a_star =AStar()
    a_star.RunAndSaveImage()
    rospy.loginfo("the distance is ")
    rospy.loginfo(mypath)
    i=0
    path = Path()
    path.header.seq = 0
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'map'
    while(i<len(mypath)):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        pose.pose.position.y = (mypath[i].y-21)/2.1
        pose.pose.position.x = (mypath[i].x-21)/2.1
        pose.pose.position.z = 0.01
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1
        path.poses.append(pose)
        i=i+1
    pose = PoseStamped()
    pose.header.seq = i
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = 'map'
    pose.pose.position.y = curposy
    pose.pose.position.x = curposx
    pose.pose.position.z = 0.01
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.insert(0,pose)
    i=i+1
    pose = PoseStamped()
    pose.header.seq = i
    pose.header.stamp = rospy.Time(0)
    pose.header.frame_id = 'map'
    pose.pose.position.y = goaly
    pose.pose.position.x = goalx
    pose.pose.position.z = 0.01
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.append(pose)
    i=i+1
    rospy.loginfo(path)
    path_pub.publish(path)
    

def mygetgoal():
    rospy.Subscriber("/course_agv/goal",PoseStamped,callbackgoal)
  

def callbackpos(data):
    global curposx
    curposx=data.transforms[0].transform.translation.x
    global curposy
    curposy=data.transforms[0].transform.translation.y
    #rospy.loginfo("posx="+str(curposx)+"posy="+str(curposy) )

def mygetposition():
    rospy.Subscriber("/tf",TFMessage,callbackpos)
    

def mygetmap():
    rospy.wait_for_service('/static_map')
    try:
        getmap = rospy.ServiceProxy('/static_map', GetMap)
        mymap = getmap().map.data
        return mymap
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def mylen(x1,y1,x2,y2):
    mydx=abs(x2-x1)
    mydy=abs(y2-y1)
    return mydx + mydy + (np.sqrt(2) - 2) * min(mydx, mydy)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = sys.maxsize

class AStar:
    def __init__(self):
        self.open_set = []  
        self.close_set = []

    def BaseCost(self, p):
        x_dis = abs(p.x-int(21+curposx*2.1+0.5)) 
        y_dis = abs(p.y-int(21+curposy*2.1+0.5))
        # Distance to start point
        return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)

    def HeuristicCost(self, p):
        x_dis = abs(int(21+goalx*2.1+0.5)  - p.x)
        y_dis = abs(int(21+goaly*2.1+0.5)  - p.y)
        # Distance to end point
        return x_dis + y_dis + (np.sqrt(2) - 2) * min(x_dis, y_dis)

    def TotalCost(self, p):
        return self.BaseCost(p) + self.HeuristicCost(p)

    def IsValidPoint(self, x, y):
        if x < 0 or y < 0:
            return False
        if x >= 43 or y >= 43:
            return False
        if mycost[x,y]<10:
            return False  
        return True

    def IsInPointList(self, p, point_list):
        for point in point_list:
            if point.x == p.x and point.y == p.y:
                return True
        return False

    def IsInOpenList(self, p):
        return self.IsInPointList(p, self.open_set)

    def IsInCloseList(self, p):
        return self.IsInPointList(p, self.close_set)

    def IsStartPoint(self, p):
        return p.x == int(21+curposx*2.1+0.5) and p.y ==int(21+curposy*2.1+0.5)

    def IsEndPoint(self, p):
        return p.x == int(21+goalx*2.1+0.5) and p.y == int(21+goaly*2.1+0.5)

    def RunAndSaveImage(self):
        start_time = time.time()
        start_point = Point(int(21+curposx*2.1+0.5), int(21+curposy*2.1+0.5))
        start_point.cost = 0
        self.open_set.append(start_point)
        while True:
            index = self.SelectPointInOpenList()
            if index < 0:
                print('No path found, algorithm failed!!!')
                return
            p = self.open_set[index]
            if self.IsEndPoint(p):
                return self.BuildPath(p,start_time)
            del self.open_set[index]
            self.close_set.append(p)
            # Process all neighbors
            x = p.x
            y = p.y
            self.ProcessPoint(x-1, y+1, p)
            self.ProcessPoint(x-1, y, p)
            self.ProcessPoint(x-1, y-1, p)
            self.ProcessPoint(x, y-1, p)
            self.ProcessPoint(x+1, y-1, p)
            self.ProcessPoint(x+1, y, p)
            self.ProcessPoint(x+1, y+1, p)
            self.ProcessPoint(x, y+1, p)


    def ProcessPoint(self, x, y, parent):
        if not self.IsValidPoint(x, y):
            return # Do nothing for invalid point
        p = Point(x, y)
        if self.IsInCloseList(p):
            return # Do nothing for visited point
        #print('Process Point [', p.x, ',', p.y, ']', ', cost: ', p.cost)
        if not self.IsInOpenList(p):
            p.parent = parent
            p.cost = self.TotalCost(p)
            self.open_set.append(p)
        
        if self.IsInOpenList(p):
            if parent.cost+mylen(p.x,p.y,parent.x,parent.y)<p.cost:
                p.parent=parent
                p.cost=parent.cost+mylen(p.x,p.y,parent.x,parent.y)

    def SelectPointInOpenList(self):
        index = 0
        selected_index = -1
        min_cost = sys.maxsize
        for p in self.open_set:
            cost = self.TotalCost(p)
            if cost < min_cost:
                min_cost = cost
                selected_index = index
            index += 1
        return selected_index

    def BuildPath(self, p, start_time):
        global mypath
        mypath = []
        while True:
            mypath.insert(0, p) # Insert first
            if self.IsStartPoint(p):
                break
            else:
                p = p.parent
        end_time = time.time()
        print('===== Algorithm finish in', int(end_time-start_time), ' seconds')

def main():
    rospy.init_node('global_planner')
    global path_pub
    path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
    time.sleep(0.5)
    global mycmap
    mycmap=mygetmap()
    global mycost
    mycost = np.mat(np.zeros((43, 43),dtype=np.int))
    global dist
    dist = np.mat(np.zeros((1849,1849),dtype=np.int))
    dist=dist+ float("Inf")
    for i in range(0, 43):
        for j in range(0,43):
            if(mycmap[mypictrans2(3*i,3*j)]<10 and mycmap[mypictrans2(3*i,3*j+1)]<10 and mycmap[mypictrans2(3*i,3*j+2)]<10 and mycmap[mypictrans2(3*i+1,3*j)]<10 and mycmap[mypictrans2(3*i+1,3*j+1)]<10 and mycmap[mypictrans2(3*i+1,3*j+2)]<10 and mycmap[mypictrans2(3*i+2,3*j)]<10 and mycmap[mypictrans2(3*i+2,3*j+1)]<10 and mycmap[mypictrans2(3*i+2,3*j+2)]<10):
                mycost[i,j] = 100
            else:
                mycost[i,j]=0
    mygetposition()
    mygetgoal()
    
    #curposx=0
    #curposy=0
    #goalx=-6
    #goaly=4
        
    rospy.spin()
    

if __name__ == '__main__':
    main()
