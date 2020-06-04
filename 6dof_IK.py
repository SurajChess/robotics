"""
inverse kinematics for 6dof and simualation of helix function as end points
Written by Akshay Khemnar while doing Internship at Robiotar Labs pvt ltd

"""
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
import matplotlib.pyplot as plt
# input the thetha4,thetha5,thetha6 for orientation of end effector
thetha_o = [0*(np.pi/180),45*(np.pi/180),0*(np.pi/180)]

# end effector position
e_p_x = list()
e_p_y = list()
e_p_z = list()

# link length
a=[1,10,10,2,2,2]

def c(t):
    return np.cos(t)

def s(t):
    return np.sin(t)

"""
n_turns is for number of helix turns
radius is for the radius of helix
cx is the x-coordinate centre point of helix loop
cy is the y-coordinate centre point of helix loop
"""
def helix (n_turns,radius,cx,cy):
    helix_thetha = np.linspace(0, n_turns * np.pi, 101)
    y = radius*s(helix_thetha) + cy
    x = radius*c(helix_thetha) + cx
    z = np.linspace(0,10,101)
    return x,y,z
e_p = helix(6,2,0,0)
e_p_x = e_p[0]
e_p_y = e_p[1]
e_p_z = e_p[2]

def homogenous_matrix(DH_p):
    t = DH_p[0]
    a = DH_p[1]
    r = DH_p[2]
    d = DH_p[3]
    homogenous_matrix = [[c(t), -s(t)*c(a),   s(t)*s(a),  r*c(t)],
                         [s(t),  c(t)*c(a),  -c(t)*s(a),  r*s(t)],
                         [   0,       s(a),        c(a),       d],
                         [   0,          0,           0,       1]]

    return np.round(homogenous_matrix,4)

def forward_kinematics(DH,initial_x,initial_y,initial_z):
    A = np.identity(4)
    n = np.shape(DH)[0]
    ox = [0]
    oy = [0]
    oz = [0]
    for i in range(n):
        Anext = homogenous_matrix(DH[i])
        A = np.matmul(A,Anext)
        ox.append(A[0][3])
        oy.append(A[1][3])
        oz.append(A[2][3])

    for i in range(n+1):
        ox[i] = ox[i]+initial_x
        oy[i] = oy[i]+initial_y
        oz[i] = oz[i]+initial_z
    return ox,oy,oz

# function to find the thetha1,thetha2,thetha3 by taking the input as link3 coordinate
# l1_l, l2_l, l3_l is the link length of first, second, and third link respectively
def i_k_first_3_joints(p_x, p_y, p_z, l1_l, l2_l, l3_l):
    thetha1_3 = [0, 0, 0]
    thetha1_3[0] = math.atan2(p_y,p_x)                                  # --- 1
    r1 = math.sqrt((p_x**2) + (p_y**2))                                # --- 2
    r2 = p_z - l1_l                                                    # --- 3
    r3 = math.sqrt((r1**2) + (r2**2))                                  # --- 4
    phi2 = math.atan2(r2,r1)                                          # --- 5
    phi1 = math.acos((l2_l*l2_l + r3*r3 - l3_l*l3_l)/(2*l2_l*r3))
    thetha1_3[1] = phi2 - phi1                                         # --- 7
    phi3 = math.acos((l2_l**2 + l3_l**2 - r3**2)/(2*l2_l*l3_l))        # --- 8
    thetha1_3[2] = 180*(np.pi/180) - phi3                              # --- 9
    return thetha1_3

DH_6_5_4 = [ [thetha_o[2], -90*(np.pi/180),    0,      a[5]+a[4]],
             [thetha_o[1],  90*(np.pi/180),    0,             0 ],
             [thetha_o[0],   0*(np.pi/180),    0,           a[3]] ]

# for plotting 3d Graph
mpl.rcParams['legend.fontsize'] = 8
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_zlabel('z')
ax.set_xlabel('x')
ax.set_ylabel('y')
elev = 45
azim = 45
ax.view_init(elev, azim)
ax.plot3D(e_p_x,e_p_y,e_p_z,label="helix")
ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)
ax.set_zlim(0, 20)
plt.pause(0.1)

for i in range(0,len(e_p_x),1):

    links_p_6_5_4 = forward_kinematics(DH_6_5_4,e_p_x[i],e_p_y[i],e_p_z[i])

    f_thetha = i_k_first_3_joints(links_p_6_5_4[0][3],links_p_6_5_4[1][3],links_p_6_5_4[2][3],a[0],a[1],a[2])

    DH_1_2_3 = [ [f_thetha[0], 90*(np.pi/180),      0,   a[0]],
                 [f_thetha[1],  0*(np.pi/180),   a[1],     0 ],
                 [f_thetha[2],  0*(np.pi/180),   a[2],     0 ] ]

    links_p_1_2_3 = forward_kinematics(DH_1_2_3,0,0,0)


    # using section formula for link5
    L_5_p_3 = list()
    L_5_p_3.append((a[4]*links_p_6_5_4[0][1]+a[4]*links_p_6_5_4[0][0])/(a[4]+a[4]))
    L_5_p_3.append((a[4]*links_p_6_5_4[1][1]+a[4]*links_p_6_5_4[1][0])/(a[4]+a[4]))
    L_5_p_3.append((a[4]*links_p_6_5_4[2][1]+a[4]*links_p_6_5_4[2][0])/(a[4]+a[4]))

    print(links_p_6_5_4)

    # for link 6
    x6 = [links_p_6_5_4[0][0],L_5_p_3[0]]
    y6 = [links_p_6_5_4[1][0],L_5_p_3[1]]
    z6 = [links_p_6_5_4[2][0],L_5_p_3[2]]
    link6 = ax.plot(x6, y6, z6,'-y', label='link 6')

    # for link 5
    x5 = [L_5_p_3[0],links_p_6_5_4[0][1]]
    y5 = [L_5_p_3[1],links_p_6_5_4[1][1]]
    z5 = [L_5_p_3[2],links_p_6_5_4[2][1]]
    link5 = ax.plot(x5, y5, z5,'-m', label='link 5')

    # for link 4
    x4 = [links_p_6_5_4[0][1],links_p_6_5_4[0][3]]
    y4 = [links_p_6_5_4[1][1],links_p_6_5_4[1][3]]
    z4 = [links_p_6_5_4[2][1],links_p_6_5_4[2][3]]
    link4 = ax.plot(x4, y4, z4,'-g', label='link 4')

    # for link 1
    x1 = [links_p_1_2_3[0][0],links_p_1_2_3[0][1]]
    y1 = [links_p_1_2_3[1][0],links_p_1_2_3[1][1]]
    z1 = [links_p_1_2_3[2][0],links_p_1_2_3[2][1]]
    link1 = ax.plot(x1, y1, z1,'-b', label='link 1')

    # for link 2
    x2 = [links_p_1_2_3[0][1],links_p_1_2_3[0][2]]
    y2 = [links_p_1_2_3[1][1],links_p_1_2_3[1][2]]
    z2 = [links_p_1_2_3[2][1],links_p_1_2_3[2][2]]
    link2 = ax.plot(x2, y2, z2,'-r', label='link 2')

    # for link 3
    x3 = [links_p_1_2_3[0][2],links_p_1_2_3[0][3]]
    y3 = [links_p_1_2_3[1][2],links_p_1_2_3[1][3]]
    z3 = [links_p_1_2_3[2][2],links_p_1_2_3[2][3]]
    link3 = ax.plot(x3, y3, z3,'-k', label='link 3')
    plt.pause(0.1)

    for handle in link6:
        handle.remove()
    for handle in link5:
        handle.remove()
    for handle in link4:
        handle.remove()
    for handle in link3:
        handle.remove()
    for handle in link2:
        handle.remove()
    for handle in link1:
        handle.remove()


plt.show()

