
"""
Written by
Suraj Borate at Robiotar Labs
"""

import numpy as np
import scipy as sc
import matplotlib.pyplot as plt
import scipy.spatial as sp
import math


def minkowski_sum(robot_x, robot_y, ob1_x, ob1_y):
    robot_inverse_x = -robot_x
    robot_inverse_y = -robot_y
    minkowski_sum_x = np.zeros([robot_x.size * ob1_x.size])
    minkowski_sum_y = np.zeros([robot_y.size * ob1_y.size])
    k = 0
    for i in robot_inverse_x:
        for j in ob1_x:
            minkowski_sum_x[k] = i + j
            k = k + 1
    k = 0
    for i in robot_inverse_y:

        for j in ob1_y:
            minkowski_sum_y[k] = i + j
            k = k + 1
    return minkowski_sum_x, minkowski_sum_y

def remove_duplicates(a):
    # b = a.tolist()
    c = list(dict.fromkeys(a))
    d = np.array(c)

    return d

def xy_to_points(x, y):
    p = [(x[i], y[i]) for i in range(0, len(x))]
    return p

def points_to_list(p):
    x = [p[i][0] for i in range(1, len(p))]
    y = [p[i][1] for i in range(1, len(p))]
    return x, y


def C_obstacle(robot,obstacle):
    robot_x = robot[0]
    robot_y = robot[1]
    vv = []
    for i in range(1,obstacle.shape[0],2):
        ob1_x = obstacle[i-1]
        ob1_y = obstacle[i]

    #ob1_x = obstacle[0]
    #ob1_y = obstacle[1]

        minkowski_sumxy_ob1 = minkowski_sum(robot_x, robot_y, ob1_x, ob1_y)

        minkowski_sum_ob1_x= minkowski_sumxy_ob1[0].tolist()
        minkowski_sum_ob1_y= minkowski_sumxy_ob1[1].tolist()
        p = xy_to_points(minkowski_sum_ob1_x,minkowski_sum_ob1_y)
        p = remove_duplicates(p)
        hull = sp.ConvexHull(p)
        v = [p[hull.vertices,0],p[hull.vertices,1]]
        vx = p[hull.vertices,0]
        vy = p[hull.vertices,1]
        vv.append(vx)
        vv.append(vy)
        plt.plot(np.append(v[0],v[0][0]),np.append(v[1],v[1][0]))

    return vv

def collision_check_point(X,Y,pointu):
    count  = 0
    for i in range(0,len(X)-1):
       ax = X[i]
       bx = X[i+1]
       ay = Y[i]
       by = Y[i+1]
       if (pointu[1] == ay or pointu[1] == by):
        pointu[1] = pointu[1] + 0.1
       else:
        #print((bx - ax)*pointu[1] - (by - ay)* pointu[0] - (ay * bx - by * ax))
        if pointu[1]>ay and pointu[1]<by :
            if ((bx-ax)*pointu[1]-(by-ay)*pointu[0]-(ay*bx-by*ax) > 0):
                count = count + 1
        if pointu[1]<ay and pointu[1]>by :
            if ((bx-ax)*pointu[1]-(by-ay)*pointu[0]-(ay*bx-by*ax) < 0):
                count = count + 1
    #print(count)

    if count % 2 == 0:
        check = 0
    else:
        check = 1
    return check


def check_collision_all_obstacles_point(c_obstacle,pointa):
    check = 2
    for i in range(0,len(c_obstacle)-1,2):
        X = np.append(c_obstacle[i],c_obstacle[i][0])
        Y = np.append(c_obstacle[i+1],c_obstacle[i+1][0])
        check = collision_check_point(X,Y,pointa)
        if check == 1:
            return check
    return check

def orientation_3points(a1,a2,a3):
    x1 = a1[0]
    y1 = a1[1]
    x2 = a2[0]
    y2 = a2[1]
    x3 = a3[0]
    y3 = a3[1]

    if (y2-y1)*(x3-x2)-(y3-y2)*(x2-x1)<0:

        return 1 # countercolockwise
    if (y2-y1)*(x3-x2)-(y3-y2)*(x2-x1)>0:

        return 2 # clockwise
    if (y2-y1)*(x3-x2)-(y3-y2)*(x2-x1)==0:

        return 0 # collinear

def onSegment_collinear(a1,a2,a3):
    if (a2[0] <= max(a1[0], a3[0]) and a2[0] >= min(a1[0], a3[0]) and a2[1] <= max(a1[1], a3[1]) and a2[1] >= min(a1[1], a3[1])):
        return 1
    else:
        return 0


def collision_check_edge(X,Y,edge1):
    c = edge1[0]
    d = edge1[1]
    # for i in range(1,(int)(hull1.size/2)):
    #    a = hull1[i-1]
    #    b = hull1[i]
    for i in range(0,len(X)-1):
       ax = X[i]
       bx = X[i+1]
       ay = Y[i]
       by = Y[i+1]
       a = [ax,ay]
       b = [bx,by]

       o1 = orientation_3points(a,b,c)
       o2 = orientation_3points(a,b,d)
       o3 = orientation_3points(c,d,a)
       o4 = orientation_3points(c,d,b)


       if o1==0 and onSegment_collinear(a,c,b)==1:
           return 1

       elif o2 == 0 and onSegment_collinear(a,d,b)==1:

           return 1

       elif o3 == 0 and onSegment_collinear(c,a,d) == 1:

           return 1

       elif o4 == 0 and onSegment_collinear(c, b, d) == 1:
           return 1


       elif o1!=o2 and o3!=o4:
           return 1
       else:
           checke = 0

    return checke

def check_collision_all_obstacles_edge(c_obstacle,edge1):
    check = 2
    for i in range(0,len(c_obstacle)-1,2):
        X = np.append(c_obstacle[i],c_obstacle[i][0])
        Y = np.append(c_obstacle[i+1],c_obstacle[i+1][0])
        check = collision_check_edge(X,Y,edge1)
        if check == 1:
            return check
    return check

def distance(x1,y1,x2,y2):
    dx = x2-x1
    dy = y2-y1

    ###
    return np.sqrt((dx*dx+dy*dy))

def generate_rrt(c_obstacle,startx,starty,goalx,goaly,xmin,xmax,ymin,ymax,DIST,ITER):

        start_node = (startx,starty)
        goal_node = (goalx,goaly)
        tree = {start_node:None}
        distance_min =100.0
        i = 0
        while i <= ITER:
            i = i + 1
            new_nodex = np.random.random() * (xmax - xmin) + xmin
            new_nodey = np.random.random() * (ymax - ymin) + ymin
            new_node = [new_nodex, new_nodey]
            new_node_tuple = (new_nodex, new_nodey)
            distance_min = 1000.0
            for old_node in tree:
                d = distance(old_node[0], old_node[1], new_node[0], new_node[1])
                if d <= distance_min:
                    distance_min = d
                    prev_node_tuple = (old_node[0],old_node[1])
                    prev_node = [old_node[0],old_node[1]]


            if distance_min>=DIST:
                theta = math.atan2(new_node[1]-prev_node[1],new_node[0]-prev_node[0])
                new_node[0] = prev_node[0]+ DIST*math.cos(theta)
                new_node[1] = prev_node[1]+ DIST*math.sin(theta)
                new_node = [new_node[0],new_node[1]]
                new_node_tuple = (new_node[0],new_node[1])
            checkp = check_collision_all_obstacles_point(c_obstacle,new_node)
            edge1 = [new_node,prev_node]

            if checkp != 1:
                checke = check_collision_all_obstacles_edge(c_obstacle,edge1)
                if checke != 1:
                    tree[new_node_tuple] = prev_node_tuple
                    plt.plot([new_node[0],prev_node[0]],[new_node[1],prev_node[1]])
                    plt.pause(0.01)
                    if distance(new_node[0],new_node[1],goalx,goaly) <= 1.0:
                        plt.plot([new_node[0],goalx],[new_node[1],goaly])
                        goal_node_tuple = (goalx,goaly)
                        tree[goal_node_tuple] = new_node_tuple
                        return tree
        return tree


def get_path_from_rrt(rrt,start_node_tuple,goal_node_tuple):
    temp = rrt.get(goal_node_tuple)
    path = [goal_node_tuple,temp]
    while(temp!=start_node_tuple):
        temp = rrt.get(temp)
        path.append(temp)
    print(path)
    pathx = []
    pathy = []
    for i in range(len(path)):
        pathx.append(path[i][0])
        pathy.append(path[i][1])
    plt.plot(pathx,pathy,'-k')


    return path



# Define obstacles

ob1_x= np.array([1,3,3,1])
ob1_y = np.array([1,1,2,2])
ob2_x = np.array([6,8,8,9])
ob2_y = np.array([6,6,7,10])

# Plot obstacles

plt.plot(np.append(ob1_x,ob1_x[0]),np.append(ob1_y,ob1_y[0]))
plt.plot(np.append(ob2_x,ob2_x[0]),np.append(ob2_y,ob2_y[0]))

# Define Robot
robot_x = np.array([0,0,-1])
robot_y = np.array([0,-1,-1])

# Plot Robot

plt.plot(np.append(robot_x,robot_x[0]),np.append(robot_y,robot_y[0]))
plt.plot([0],[0],'*')

robot = np.array([robot_x,robot_y])
obstacle = np.array([ob1_x,ob1_y,ob2_x,ob2_y])

# find c_obstacle so that we can now do motion planning as if it is a point robot

c_obstacle = C_obstacle(robot,obstacle)

# define arena

xmin = -2
ymin = -2
xmax = 8
ymax = 8

#Define Start and goal
startx = 0
starty = 2
goalx = 6
goaly = 4
goal_node_tuple =(goalx,goaly)
start_node_tuple =(startx,starty)
#plotting start and goal points
plt.plot(startx,starty,'og')
plt.plot(goalx,goaly,'or')

ITER = 10000
DIST = 1

rrt = generate_rrt(c_obstacle,startx,starty,goalx,goaly,xmin,xmax,ymin,ymax,DIST,ITER)
print(rrt)

get_path_from_rrt(rrt,start_node_tuple,goal_node_tuple)
plt.show()

