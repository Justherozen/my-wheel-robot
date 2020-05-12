#!/usr/bin/env python
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
import time
import numpy as np
global mycmap
mycmap=[]
def mypictrans2(x, y):
    picpos =int(129 * x + y)
    return picpos

def mypictrans(x, y):
    picpos =int(43 * x + y)
    return picpos

def shortPath_Dijkstra(graph, v0, v1):
    n = len(graph)
    global final
    global P
    global D
    global k
    k=0
    final, P, D = [0] * n, [0] * n, [0] * n
    for i in range(n):  
        D[i] = graph[v0,i]
    D[v0] = 0
    final[v0] = 1
    for v in range(1, n):
        min = float("Inf")
        for w in range(0, n):
            if  (final[w]==0 and D[w] < 100000 and D[w]<min and mycost[w//43,w%43]>10) :
                k = w
                min = D[w]

        final[k] = 1
        for w in range(0, n):
            if (100>graph[k,w]>5 and final[w]==0 and min + graph[k,w] < D[w]):
                D[w] = min + graph[k,w]
                P[w] = k
        #print(v)
    return D[v1]

def callbackgoal(data):
    global goalx
    goalx=data.pose.position.x
    global goaly
    goaly=data.pose.position.y
    rospy.loginfo("goalx="+str(goalx)+"goaly="+str(goaly) )
    rospy.loginfo("the distance is"+str(1))
    rospy.loginfo(shortPath_Dijkstra(dist,mypictrans(int(21+curposy*2.1),int(21+curposx*2.1)),mypictrans(int(21+goaly*2.1),int(21+goalx*2.1))))
    i=0
    rx = [0,-1,-2,-3]
    ry = [0,1,0,1]
    path = Path()
    path.header.seq = 0
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'map'
    pathdata=mypictrans(int(21+goaly*2.1),int(21+goalx*2.1))
    while(P[pathdata]!=mypictrans(int(21+curposy*2.1),int(21+curposx*2.1)) and P[pathdata]!=0):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        pose.pose.position.y = (pathdata//43-21)/2.1+0.2
        pose.pose.position.x = (pathdata%43-21)/2.1+0.2
        pathdata=P[pathdata];
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
    pathdata=P[pathdata];
    pose.pose.position.z = 0.01
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1
    path.poses.append(pose)
    i=i+1
    path.poses.reverse()
    path.poses=path.poses[1:]
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

class GlobalPlanner:
    def __init__(self):
        # TODO  : 
        #   tf listener : 
        #   publisher   : /course_agv/global_path
        #   subscriver  : /course_agv/goal
        #   service     : /course_agv/global_plan
        #   initialize for other variable
        
        #example
       

        pass
    

    
    def getpath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'


        


def main():
    rospy.init_node('global_planner')
    global path_pub
    path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)
    gp = GlobalPlanner()
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
            #print(mycost[i,j])
    for i in range(0, 43):
        for j in range(0, 43):
            dist[mypictrans(i, j),mypictrans(i, j)] = 0
            
            if (43 > i> 0 and 42 > j >=0 and mycost[i, j] > 10 and mycost[i - 1, j + 1] > 10 ):
                dist[mypictrans(i, j),mypictrans( i - 1,j + 1)] = dist[mypictrans(i - 1,j + 1),mypictrans(i, j)] = 14
            #else:
             #   dist[mypictrans(i, j),mypictrans(i - 1,j + 1)] = dist[mypictrans(i - 1,j + 1),mypictrans(i, j)] = -1
            if (43 > i > -1 and 42 > j > -1 and mycost[i, j]>10 and mycost[i, j + 1]>10 ):
                dist[mypictrans(i, j),mypictrans(i, j + 1)] = dist[mypictrans(i, j + 1),mypictrans(i, j)] = 10
            #else:
             #   dist[mypictrans(i, j),mypictrans(i, j + 1)] = dist[mypictrans(i, j + 1),mypictrans(i, j)] = -1

            if (42 > i > -1 and 42 > j > -1 and mycost[i, j]>10 and mycost[i + 1, j + 1]>10 ):
                dist[mypictrans(i, j),mypictrans(i + 1,j + 1)] = dist[mypictrans(i + 1,j + 1),mypictrans(i, j)] = 14
           # else:
            #    dist[mypictrans(i, j),mypictrans(i + 1,j + 1)] = dist[mypictrans(i + 1,j + 1),mypictrans(i, j)] = -1

            if (42 > i > -1 and 43 > j > -1 and mycost[i, j]>10 and mycost[i + 1, j]>10):
                dist[mypictrans(i, j),mypictrans(i + 1, j)] = dist[mypictrans(i + 1, j),mypictrans(i, j)] = 10
            #else:
             #   dist[mypictrans(i, j),mypictrans(i + 1, j)] = dist[mypictrans(i + 1, j),mypictrans(i, j)] = -1
    mygetposition()
    mygetgoal()
    
    #curposx=0
    #curposy=0
    #goalx=-6
    #goaly=4
        
    rospy.spin()
    

if __name__ == '__main__':
    main()
