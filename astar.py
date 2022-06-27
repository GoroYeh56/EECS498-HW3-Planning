import numpy as np
from utils import get_collision_fn_PR2, load_env, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, find, get_closest_edge_point, get_joint_positions, wait_if_gui, set_joint_positions, joint_from_name, get_link_pose, link_from_name
from pybullet_tools.pr2_utils import PR2_GROUPS
import time


## TODO:
## 1. Computation time
## 2. Path Cost
## 3. Screenshot * 2

### YOUR IMPORTS HERE ###
from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker, draw_line
from pybullet_tools.utils import connect, disconnect, wait_if_gui, wait_for_user, joint_from_name, get_joint_info, get_link_pose, link_from_name
from queue import PriorityQueue
import math

class Node:
    def __init__(self, x_in,y_in,theta_in, g):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.g = g
        # self.parentid = parelntid_in

    def printme(self):
        # print("\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid)
        print("\tNode:", "x =", self.x, "y =",self.y, "theta =", self.theta,  "g =",self.g)
    def getXYTg(self):
        return self.x, self.y, self.theta, self.g

# Cost from n to m
def getG(nx,ny, nt, mx, my, mt):
    g = pow( (pow((mx-nx),2) + pow((my - ny),2) + min(abs(mt-nt),  2*np.pi-abs(mt-nt))), 0.5  )
    return g

def getH(x,y, theta, gx, gy, gtheta):
    h  = pow( (pow((x-gx),2) + pow((y - gy),2) + min(abs(theta-gtheta),  2*np.pi-abs(theta-gtheta))), 0.5  )
    return h

def ComputeF(lastx, lasty, lastt, lastg, x, y, t, gx, gy, gt):
    g = lastg  + getG(lastx, lasty, lastt, x, y, t)
    h = getH(x,y,t, gx, gy, gt)
    f = g + h
    # epsilon = 0.0001 # avoid divided by 0
    # return 1/(f+epsilon)
    return f

def reachGoal(x, y, t, gx, gy, gt, goal_threshold):
    if getH(x,y, t, gx, gy, gt) <= goal_threshold:
        return True
    else:
        return False


#########################

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2doorway.json')

    # define active DoFs
    base_joints = [joint_from_name(robots['pr2'], name) for name in PR2_GROUPS['base']]

    # Example use of setting body poses
    # set_pose(obstacles['ikeatable6'], ((0, 0, 0), (1, 0, 0, 0)))
    
    start_config = tuple(get_joint_positions(robots['pr2'], base_joints))
    goal_config = (2.6, -1.3, -np.pi/2) # x, y, theta
    # goal_config = (2.6, 1, -np.pi/2) # x, y, theta
    path = []
    start_time = time.time()
    ### YOUR CODE HERE ###
 
    collision_fn = get_collision_fn_PR2(robots['pr2'], base_joints, list(obstacles.values()))
    def collision(x, y, yaw):
        return collision_fn((x, y, yaw))

    PathCost = -1

    TEST_LENGTH = 5000
    # GOAL_THRESHOLD = 1.3 # 1.3: 1689 iter
    # GOAL_THRESHOLD = 1.2 # 1.2 : 2980, (1.5, 1.1, -np.pi/2), 523.58 sec
    # GOAL_THRESHOLD = 1 # 2986 reach (1,7, 1.1,  -1.57) / 6697 1.9, -0.6, -pi/2, 1595 second
    # GOAL_THRESHOLD = 0.5 # 3000, (2.2, 1.1, -pi) 354 second / 7578 (2.3, -0.9, -pi/2) 620 second
    GOAL_THRESHOLD = 0.1 # 3047 (2.5, 1.1, -pi) 261 sec / 8558 iter,(2.5, -1.2999, -pi/2) 706 second
    step_size_x = 0.15 # Can be larger: Maybe 0.2?
    step_size_y = 0.15

    # Run A*
    # q is our open set
    q = PriorityQueue()
    start_x, start_y, start_t = start_config
    start_g = 0 # cost of start node
    gx, gy, gt = goal_config

    ### Draw start and goal pose
    sphere_radius = 0.1
    sphere_color_r = (1, 0, 0, 1) # R, G, B, A 
    print("Start Pose: ", start_config)
    draw_sphere_marker((start_x, start_y, 0), sphere_radius, sphere_color_r)
    print("Gaol Pose: ", goal_config)
    sphere_color_g = (0, 1, 0, 1) # R, G, B, A 
    draw_sphere_marker((gx, gy, 0), sphere_radius, sphere_color_g)


    def node2id(node):
        return str(node.x) + str(node.y)+str(node.theta)
    id = 0
    start = Node(start_x, start_y, start_t, start_g)
    priority = ComputeF(start_x,start_y,start_t, start_g, start_x, start_y, start_t, gx, gy, gt)
    q.put( (priority, id, start)  )

    # Below two are for visualization used
    Astar_path_circles = [] # x,y, z=0.3
    Astar_obs_nodes = []  # red
    Astar_free_nodes = [] # blue

    open = []
    closed = []
    Parent = {}
    gcost = {}
    fcost = {}
    root_parent = Node(0,0,0,0)
    Parent[start] = root_parent # root of start node is -1.
    open.append(node2id(start)) # means visited start
    # Type: Node(has x, y, z)
    def ExtractPath(node):
        while Parent[node] !=  root_parent:
            # path.append((node.x, node.y, 0.3)) # z=0
            path.append((node.x, node.y, node.theta))
            Astar_path_circles.append((node.x, node.y, 0.3))
            node = Parent[node]
        path.reverse()


    directions4 = [(1,0), (-1,0), (0,1), (0,-1)]
    directions8 = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1),  (1,-1), (-1,-1)]
    # degrees = [0, np.pi/4, np.pi/2, np.pi*3/4, np.pi, -np.pi*3/4, -np.pi/2, -np.pi/4]
    degrees = [0, np.pi/2, np.pi, -np.pi/2]
    iter = 0
    findpath = False

    FACTOR = 200
    FOUR_CONNECTED = True
    DRAW_EXPLORATION_NODES = True
    
    if FOUR_CONNECTED:
        directions = directions4
    else:
        directions = directions8

    while not q.empty() and iter<TEST_LENGTH:
        iter += 1
        if iter%FACTOR==0:
            print("iter: ",iter)
        curNode = q.get()
        # print("Priority:", curNode[0])
        # curNode[2].printme()

        nx, ny, nt, ng = curNode[2].getXYTg()

        # if reach goal (within certain threshold)
        if reachGoal(nx, ny, nt, gx, gy, gt, GOAL_THRESHOLD):
            print(f"At iteration {iter} reach goal!")
            ExtractPath(curNode[2])
            PathCost = curNode[0] # Its priority
            print("Final x,y,theta: ",nx, ny, nt)
            findpath = True
            break

        # Place current node from openlist to closedlist
        open.remove(node2id(curNode[2]))
        # closed.append(Node(nx, ny, nt, ng))
        closed.append(node2id(curNode[2]))

        # KEY : didn't correctly process as 'set'
        # print("start in closed? ", node2id(start) in closed)

        for dir in directions:
            dx, dy = dir
            mx = nx + dx*step_size_x
            my = ny + dy*step_size_y
            for orientation in degrees:
                mt = orientation
                mg = ng + getG(nx, ny, nt, mx, my, mt)
                id = id+1
                nextNode = Node(mx, my, mt, mg)
                nextNodeid = node2id(nextNode)
                # mt = nt ### TODO : Theta DOF!
                if  nextNodeid not in closed and not collision(mx, my, mt): # visit and add them to q (open set)
                    Astar_free_nodes.append((mx,my,0))  
                    priority = ComputeF(nx, ny, nt, ng, mx, my, mt, gx, gy, gt)
                    new_g = ng + getG(nx, ny, nt, mx, my, mt)
                    change_parent = False

                    #q.put( (priority, id, nextNode))
                    if nextNodeid not in open:
                        change_parent = True
                        q.put( (priority, id, nextNode)) 
                        open.append(nextNodeid)                   
                    elif new_g < gcost[nextNodeid]:
                        change_parent = True
                    if change_parent:
                        Parent[nextNode] = curNode[2]
                        gcost[nextNodeid] = new_g
                        fcost[nextNodeid] = ComputeF(nx, ny, nt, ng, mx, my, mt, gx, gy, gt)

                elif nextNodeid not in closed and collision(mx,my,mt):
                    Astar_obs_nodes.append((mx,my,0))

        # print("Priority:", next_item[0])
        # next_item[2].printme()

    
    if not findpath:
        print("No availble path within this configuratoin\n")

    # print(f"Path: {path}")
    print("Path:")
    fmt="(%.3f, %.3f, %.3f)"
    for pose in path:
        print(fmt%pose)

    if FOUR_CONNECTED:
        print("4-connected:")
    else:
        print("8-connected:")
    print("Planner run time: ", time.time() - start_time)
    print("Path cost: ", PathCost)


    ############### Draw the path ################
    # print("Press enter to generate path spheres")
    # wait_for_user()
    print("Now draw path...")
    sphere_radius = 0.1
    sphere_color_black = (0, 0, 0, 1) # R, G, B, A
    # for pose in path:
    Astar_path_circles.reverse()
    for pose in Astar_path_circles:
        draw_sphere_marker(pose, sphere_radius, sphere_color_black)

    print("=======================================")
    print("Now executing the path:")
    print("path: ",path)
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)

    ######################

    Astar_free_nodes = list(set(Astar_free_nodes))
    Astar_obs_nodes = list(set(Astar_obs_nodes))

    print("# Free A* node: ",len(Astar_free_nodes))
    print("# Obs A* node: ",len(Astar_obs_nodes))


    if DRAW_EXPLORATION_NODES:
        print("Plot Free space node...")
        sphere_color_blue = (0, 0, 1, 1)
        for pose in Astar_free_nodes:
            draw_sphere_marker(pose, sphere_radius, sphere_color_blue)
        
        print("Plot obstacle nodes: ...")
        for pose in Astar_obs_nodes:
            draw_sphere_marker(pose, sphere_radius, sphere_color_r)        
        print("=======================================")

    # ######################
    # Execute planned path
    execute_trajectory(robots['pr2'], base_joints, path, sleep=0.2)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()
