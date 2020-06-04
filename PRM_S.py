"""
Written by
Suraj Borate at Robiotar Labs
"""

import numpy as np
#import scipy as sc
import matplotlib.pyplot as plt
import scipy.spatial as sp

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


def generate_n_nodes(xmin,xmax,ymin,ymax,c_obstacle,NMAX):
    n = 0
    verticesx = []
    verticesy = []
    while n<=NMAX-1:
        nodex1 = np.random.random()*(xmax-xmin) + xmin
        nodey1 = np.random.random()*(ymax-ymin) + ymin
        node = nodex1,nodey1
        check = check_collision_all_obstacles_point(c_obstacle,node)
        if check == 1:
            n = n
        else:
            n = n+1
            verticesx.append(nodex1)
            verticesy.append(nodey1)
    return verticesx,verticesy

def distance(x1,y1,x2,y2):
    dx = x2-x1
    dy = y2-y1
    return np.sqrt((dx*dx+dy*dy))

def generate_graph_PRM(verticesx,verticesy,cobstacle,DIST_RANGE):
    g = np.zeros((len(verticesx)+2,len(verticesx)+2))-1
    graph = {}

    for i in range(len(verticesx)):
        graphj = {}
        for j in range(len(verticesx)):
            d = distance(verticesx[i], verticesy[i], verticesx[j], verticesy[j])
            if d<=DIST_RANGE:
                edge1 = [[verticesx[i],verticesy[i]],[verticesx[j],verticesy[j]]]
                checke = check_collision_all_obstacles_edge(cobstacle,edge1)
                if checke != 1:
                    #g[i][j] = d
                    graphj[j] = d
        graph[i] = graphj
    return graph


def add_start_goal(cobstacle,verticesx,verticesy,graph,startx,starty,goalx,goaly):

    sgx = [startx,goalx]
    sgy = [starty,goaly]

    for i in range(len(sgx)):
        graphj ={}
        for j in range(len(verticesx)):
            d = distance(sgx[i], sgy[i], verticesx[j], verticesy[j])
            if d<=DIST_RANGE:
                edge1 = [[sgx[i],sgy[i]],[verticesx[j],verticesy[j]]]
                checke = check_collision_all_obstacles_edge(cobstacle,edge1)
                if checke != 1:
                    graphj[j] = d
                    graph[j][NMAX + i] = d
                    #graph[NMAX+i] = d
                    plt.plot([sgx[i], verticesx[j]], [sgy[i], verticesy[j]])


        graph[NMAX + i] = graphj


    return graph


def dijkstra(graph,start,goal):

    shortest_distance = {} #dictionary to record the cost to reach to node. We will constantly update this dictionary as we move along the graph.
    track_predecessor = {} #dictionary to keep track of path that led to that node.
    unseenNodes = graph #to iterate through all nodes
    infinity = 5000 #infinity can be considered a very large number
    track_path = [] #dictionary to record as we trace back our journey



# =============================================================================
# Initially we want to assign 0 as the cost to reach to source node and infinity as cost to all other nodes
# =============================================================================

    for node in unseenNodes:
        shortest_distance[node] = infinity

    shortest_distance[start] = 0

# =============================================================================
# The loop will keep running until we have entirely exhausted the graph, until we have seen all the nodes
# =============================================================================
# =============================================================================
# To iterate through the graph, we need to determine the min_distance_node every time.
# =============================================================================

    while unseenNodes:
        min_distance_node = None

        for node in unseenNodes:
            if min_distance_node is None:
                min_distance_node = node

            elif shortest_distance[node] < shortest_distance[min_distance_node]:
                min_distance_node = node

# =============================================================================
# From the minimum node, what are our possible paths
# =============================================================================

        path_options = graph[min_distance_node].items()


# =============================================================================
# We have to calculate the cost each time for each path we take and only update it if it is lower than the existing cost
# =============================================================================

        for child_node, weight in path_options:

            if weight + shortest_distance[min_distance_node] < shortest_distance[child_node]:

                shortest_distance[child_node] = weight + shortest_distance[min_distance_node]

                track_predecessor[child_node] = min_distance_node

# =============================================================================
# We want to pop out the nodes that we have just visited so that we dont iterate over them again.
# =============================================================================
        unseenNodes.pop(min_distance_node)



# =============================================================================
# Once we have reached the destination node, we want trace back our path and calculate the total accumulated cost.
# =============================================================================

    currentNode = goal

    while currentNode != start:

        try:
            track_path.insert(0,currentNode)
            currentNode = track_predecessor[currentNode]
        except KeyError:
            print('Path not reachable')
            break
    track_path.insert(0,start)


# =============================================================================
#  If the cost is infinity, the node had not been reached.
# =============================================================================
    if shortest_distance[goal] != infinity:
        print('Shortest distance is ' + str(shortest_distance[goal]))
        print('And the path is ' + str(track_path))
        print(track_path)
        return(track_path)
    else:
        return [0]



def get_path_coordinates(path,verticesx,verticesy,startx,starty,goalx,goaly):
    if path == [0]:
        print("no path found. Please Change Parameters ")
        return [0,0]

    verticesx.append(startx)
    verticesy.append(starty)
    verticesx.append(goalx)
    verticesy.append(goaly)
    # Plot path
    pathx =[]
    pathy =[]
    for i in range(len(path)-1):
        edgex = [verticesx[path[i]],verticesx[path[i+1]]]
        edgey = [verticesy[path[i]],verticesy[path[i+1]]]
        #plt.plot(edgex,edgey,'-k')
        pathx.append(verticesx[path[i]])
        pathy.append(verticesy[path[i]])
    pathy.append(goaly)
    pathx.append(goalx)
    return pathx,pathy


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

# Maximum nodes in PRM
NMAX = 20

# Max edge length

DIST_RANGE = 5

# generate nodes in free space

verticesx,verticesy = generate_n_nodes(xmin,xmax,ymin,ymax,c_obstacle,NMAX)

# generate graph in free space

g = generate_graph_PRM(verticesx, verticesy, c_obstacle, DIST_RANGE)

plt.axis([xmin,xmax,ymin,ymax])

# plot nodes

plt.plot(verticesx,verticesy,'*')


# plot Probabilistic Road Map

for i in range(len(verticesx)):
    for j in range(len(verticesx)):
        if i in g.keys() and j in g[i].keys():
            plt.plot([verticesx[i],verticesx[j]],[verticesy[i],verticesy[j]])
            plt.pause(0.01)
            #plt.pause(001)



#Define Start and goal
startx = 0
starty = 2
goalx = 6
goaly = 4

#plotting start and goal points
plt.plot(startx,starty,'og')
plt.plot(goalx,goaly,'or')

newgraph=add_start_goal(c_obstacle,verticesx,verticesy,g,startx,starty,goalx,goaly)


# Apply search to find optimal path

path = dijkstra(newgraph,NMAX,NMAX+1)


pathx,pathy = get_path_coordinates(path,verticesx,verticesy,startx,starty,goalx,goaly)
plt.plot(pathx,pathy,'-k')
plt.pause(0.01)
print(newgraph)

plt.show()














