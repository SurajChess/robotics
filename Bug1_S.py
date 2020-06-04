


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
    return np.sqrt((dx*dx+dy*dy))

def move_to_goal(start,goal,c_obstacle,linstep,pathx,pathy):
    xold = start[0]
    yold = start[1]

    while True:
        angle = math.atan2(goal[1]-yold,goal[0]-xold)
        xnew = xold+linstep*math.cos(angle)
        ynew = yold+linstep*math.sin(angle)
        if check_collision_all_obstacles_point(c_obstacle, [xnew,ynew])==1:
            hitx = xold
            hity = yold
            return hitx,hity,angle,pathx,pathy
        if distance(goal[0],goal[1],xold,yold)<=0.5:
            print("reached")
            return xold,yold,angle,pathx,pathy
        xold=xnew
        yold=ynew
        pathx.append(xold)
        pathy.append(yold)

        plt.plot(xnew,ynew,'og')
        plt.pause(0.01)

def turntoavoid(c_obstacle,hitx,hity,turnstep,linstep,angle):

    xold = hitx
    yold = hity
    while True:

        if distance(goal[0],goal[1],xold,yold)<=0.5:
            print('reached,turntoavoid')
            return angle
        angle = angle + turnstep
        xnew = xold + linstep*math.cos(angle)
        ynew = yold + linstep * math.sin(angle)
        edge1 = [[xnew,ynew],[xold,yold]]
        if check_collision_all_obstacles_edge(c_obstacle, edge1) != 1:
           return angle

def follow_obstacle(c_obstacle,goal,hitx,hity,freeangle,linstep,turnstep):

    k = 0
    xold = hitx
    yold = hity
    angle = freeangle
    D = distance(goal[0],goal[1],hitx,hity)
    closestx =xold
    closesty = yold
    index_closest = 0
    moved_away = 0
    while True:
        xnew = xold + linstep * math.cos(freeangle)
        ynew = yold + linstep * math.sin(freeangle)
        xold = xnew
        yold = ynew
        pathx.append(xold)
        pathy.append(yold)
        plt.plot(xold,yold,'or')
        plt.pause(0.01)

        if distance(xold,yold,hitx,hity)>0.5 and moved_away == 0:
            moved_away =1
            print(moved_away)

        if distance(xold,yold,goal[0],goal[1])<=D:
            D = distance(xold,yold,goal[0],goal[1])
            closestx = xold
            closesty = yold
            index_closest = len(pathx)-1

        if distance(goal[0],goal[1],xold,yold)<=0.5:
            print("reached,follow_obstacle")
            return xold,yold,angle,pathx,pathy,index_closest
        while True:
            xnew = xold + linstep * math.cos(freeangle)
            ynew = yold + linstep * math.sin(freeangle)
            if check_collision_all_obstacles_point(c_obstacle, [xnew,ynew]) == 1:
                freeangle = freeangle + turnstep
                break
            else:
                freeangle = freeangle - turnstep


            if distance(xold,yold,hitx,hity)<=0.5and moved_away == 1 :
                moved_away = 0
                print(3)
                return closestx,closesty,freeangle,pathx,pathy,index_closest

def move_to_takeoff(pathx,pathy,index_closest):
    pathrevx = pathx[index_closest:len(pathx)-1]
    path_to_closest_x = pathrevx[::-1]
    pathrevy = pathy[index_closest:len(pathy) - 1]
    path_to_closest_y = pathrevy[::-1]
    for i in range(len(path_to_closest_x)-1):
        pathx.append(path_to_closest_x[i])
        pathy.append(path_to_closest_y[i])
    for i in range(len(path_to_closest_x)-1):
        plt.plot(path_to_closest_x[i],path_to_closest_y[i],'om')
        plt.pause(0.01)
    xold = path_to_closest_x[len(path_to_closest_x)-1]
    yold = path_to_closest_y[len(path_to_closest_y)-1]

    return xold,yold,pathx,pathy



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
xmax = 12
ymax = 12

#Define Start and goal
startx = 0
starty = 2
goalx = 12.3
goaly = 12.3

start = [startx,starty]
xold = startx
yold = starty
goal = [goalx,goaly]

plt.plot(startx,starty,'og')
plt.plot(goalx,goaly,'or')

pathx = []
pathy = []

di = distance(goal[0], goal[1], xold, yold)
linstep=0.5
turnstep=0.2
reached = 0
def Bug1(start,goal,c_obstacle,pathx,pathy,linstep=0.2,turnstep=0.01):
    xold = start[0]
    yold = start[1]
    while (distance(goal[0],goal[1],xold,yold)>0.2):

        xold,yold,angle,pathx,pathy=move_to_goal(start,goal,c_obstacle,linstep,pathx,pathy)
        angle = turntoavoid(c_obstacle,xold,yold,turnstep,linstep,angle)
        closestx,closesty,freeangle,pathx,pathy,index_closest=follow_obstacle(c_obstacle,goal,xold,yold,angle,linstep,turnstep)
        plt.plot(closestx, closesty, 'ob')
        if distance(goal[0],goal[1],xold,yold)<=0.5:
            return pathx,pathy
        xold,yold,pathx,pathy = move_to_takeoff(pathx,pathy,index_closest)
        start = [xold,yold]


Bug1(start,goal,c_obstacle,pathx,pathy,linstep,turnstep)
plt.plot(pathx,pathy,'-k')
print("reached goal")
plt.show()