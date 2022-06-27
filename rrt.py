from posixpath import join
import numpy as np
from utils import load_env, get_collision_fn_PR2, execute_trajectory, draw_sphere_marker
from pybullet_tools.utils import connect, disconnect, draw_circle, find, get_movable_joint_descendants, wait_for_user, wait_if_gui, joint_from_name, get_joint_positions, set_joint_positions, get_joint_info, get_link_pose, link_from_name
# import random
### YOUR IMPORTS HERE ###
import random as rd
import time
from decimal import Decimal
from copy import deepcopy


def round_if_float(value):
    if isinstance(value, float):
        return Decimal(str(value)).quantize(Decimal('1.00'))
    else:
        return value

def rounded_tuple(tup):
    return tuple(round_if_float(value) for value in tup)

#########################


joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint')

def main(screenshot=False):
    # initialize PyBullet
    connect(use_gui=True)
    # load robot and obstacle resources
    robots, obstacles = load_env('pr2table.json')

    # define active DoFs
    joint_names =('l_shoulder_pan_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_upper_arm_roll_joint','l_forearm_roll_joint','l_wrist_flex_joint')
    joint_idx = [joint_from_name(robots['pr2'], jn) for jn in joint_names]

    # parse active DoF joint limits
    joint_limits = {joint_names[i] : (get_joint_info(robots['pr2'], joint_idx[i]).jointLowerLimit, get_joint_info(robots['pr2'], joint_idx[i]).jointUpperLimit) for i in range(len(joint_idx))}



    collision_fn = get_collision_fn_PR2(robots['pr2'], joint_idx, list(obstacles.values()))
    # Example use of collision checking
    # print("Robot colliding? ", collision_fn((0.5, 1.19, -1.548, 1.557, -1.32, -0.1928)))

    start_config = tuple(get_joint_positions(robots['pr2'], joint_idx))
    goal_config = (0.5, 0.33, -1.548, 1.557, -1.32, -0.1928)
    path = []

    ### YOUR CODE HERE ###
    start_time = time.time()

    # print("joint limits: ")
    # for i in range(len(joint_limits)):
    #     print(joint_limits[joint_names[i]])
    # print(joint_limits)


    joints_start = [joint_limits[joint_names[i]][0] for i in range(len(joint_limits))]
    joints_scale = [abs(joint_limits[joint_names[i]][0] - joint_limits[joint_names[i]][1]) for i in range(len(joint_limits))]
    print("joint start: ",joints_start)
    print("joint scale: ",joints_scale)
    # print("joints scale: ",joints_scale)

    ###### Initialize Parameters ######
    # KEY: node : a tuple, NOT list! 
    step_size = 0.05 #rad for each joint (revolute)
    goal_bias_prob = 0.1 # goal_bias: 10%
    goal_node = goal_config
    root_parent = (-1,-1,-1,-1,-1,-1)
    # Total: 6 DOF 
    K = 3000  # 10000 nodes iter: 81, rrt# 987
    rrt = [start_config]     # a list of config_nodes
    parents = {} # a dictionary key: tuple(a config), value: tuple(parent's config)
    parents[start_config] = root_parent

    # goal_threshold = 0.5
    goal_threshold = 0.4
    findpath = False

    def collision(robot_config):
        return collision_fn(robot_config)

    def distance(n1, n2):
        dist = 0
        weights = [0.1, 2.5, 0.6, 0.4, 0.3, 0.1]
        for i in range(len(joint_limits)):
            dist += abs( n1[i] - n2[i])*weights[i]
        return dist # Sum of angle error TODO: Distance Metrics! 6DOF

    def sample_node(goal_node, goal_bias_prob):
        # r = rd.random() # float [0.0, 1.0)
        r = np.random.random(1)
        if r <= goal_bias_prob:
            return goal_node 
        else:
            random_config = []
            rand_angles = np.random.rand(6)
            for i in range(len(joint_limits)):
                # random_angle = rd.random() # TODO: uniform random
                random_config.append(joints_start[i] + rand_angles[i] * joints_scale[i])      
            
            return tuple(random_config)

    def find_nearest_neighbor(rand_node):
        distances = []
        for node in rrt:
            distances.append(distance(node, rand_node))
        min_index = np.argmin(distances)
        return rrt[min_index]

    def direction(config1, config2):
        # return the direction of each 6 DOF joint (a list)
        dirs = []
        for i in range(len(joint_limits)):
            if config1[i] > config2[i]:
                dirs.append(-1) # clockwise
            elif config1[i] <= config2[i]:
                dirs.append(+1) # counter-clockwise
            # else:
                # dirs.append(0)
        return dirs
        
    curmincost = distance(start_config,goal_config)
    def rrt_connect(near_node, rand_node, step_size, curmincost):
        dirs = direction(near_node,rand_node)

        canbreak = True # assume go to rand and can break!
        new_node = [0, 0, 0, 0, 0, 0]
        for i in range(len(joint_limits)):
            if abs(rand_node[i] - near_node[i]) <= step_size:
                new_node[i] = rand_node[i]
            else:
                new_node[i] = near_node[i] + dirs[i]*step_size
                canbreak = False
        if(canbreak):
            if distance(goal_node, new_node) < curmincost:
                curmincost = distance(goal_node, new_node)
                print("update cur min cost: ",curmincost)
            rrt.append(rand_node)
            parents[rand_node] = near_node
            return rrt[-1], curmincost

        while not collision(new_node) and in_limit(new_node) and not canbreak:
            if distance(goal_node, new_node) < curmincost:
                curmincost = distance(goal_node, new_node)
                print("update cur min cost: ",curmincost)

            rrt.append(tuple(new_node))
            parents[tuple(new_node)] = tuple(near_node)
            near_node = tuple(new_node) # update parent
            
            canbreak = True
            for i in range(len(joint_limits)):
                if abs(rand_node[i] - near_node[i]) <= step_size:
                    new_node[i] = rand_node[i]
                    # print("joint ",i," reach rand_node")        
                else:
                    new_node[i] = near_node[i] + dirs[i]*step_size
                    canbreak = False

            if canbreak:
                rrt.append(tuple(new_node))
                parents[tuple(new_node)] = near_node
                break

        return rrt[-1], curmincost

    def step(near_node, rand_node, step_size):
        dirs = direction(near_node, rand_node)
        new_node = tuple(near_node[i] + dirs[i]*step_size for i in range(len(joint_limits))) # stear ONE STEP from near to rand 'direction'!
        return new_node


    
    def reach_goal(goal_node, node, goal_threshold):
        # print("Distance from cur pose to goal config: ",distance(goal_node, node))
        return distance(goal_node, node) <= goal_threshold
    
    def extract_path(parents, node, root_parent, debug=False):
        path = []
        while parents[node]!= root_parent:
            if debug:
                print("Node:   ",rounded_tuple(node))
                print("Parent: ",rounded_tuple(parents[node]))
            path.append(node)
            node = parents[node]
        path.reverse()
        return path


    def in_limit(config):
        for i in range(len(joint_limits)):
            if (config[i] < joint_limits[joint_names[i]][0] or \
               config[i] > joint_limits[joint_names[i]][1]) and i != 4: # KEY: joint 4 has no limit!
               print("joint ",i," out of bound.",joint_limits[joint_names[i]], " angle: ",config[i])
               return False
        return True


    print("=================================")
    print("Start config: ",start_config)
    print("Goal config: ",goal_config)
    print("Starting Error: ",distance(start_config,goal_config))
    print("RRT algorithm start: ...\n")

    FACTOR = 50
    final_node = (-1, -1, -1, -1, -1, -1) # super important
    ################ RRT Algorithm here #####################
    for i in range(K): # from 0 to K-1
        if i% FACTOR ==0:
            print("iter ",i)
        rand_node = sample_node(goal_node, goal_bias_prob) # (with 0.1 goal bias)
        nearest_node = find_nearest_neighbor(rand_node)
        new_node, curmincost = rrt_connect(nearest_node, rand_node, step_size, curmincost)

        if reach_goal(goal_node, new_node, goal_threshold):
            findpath = True
            print("Reach goal! At iter ",i," # nodes in rrt: ", len(rrt))
            print("Final pose: ",new_node)
            final_node = new_node
            path = extract_path(parents, new_node, root_parent)
            break

    if not findpath:
        print("No path found under current configuratoin")

    print("Now executing first found path: ")    

    print("Draw left end-effector position (red sphere):")
    radius = 0.03
    color = (1, 0, 0, 1)
    PR2 = robots['pr2']
    for pose in path:
        set_joint_positions(PR2, joint_idx, pose)
        ee_pose = get_link_pose(PR2, link_from_name(PR2, 'l_gripper_tool_frame'))
        draw_sphere_marker(ee_pose[0], radius, color)

    print("Planner run time: ", time.time() - start_time)

    print("Run Shortcut Smoothing Algorithm for 150 iterations...")

    def try_shortcut(parents, node1, node2, step_size):
        dirs = direction(node1,node2)

        new_nodes = []
        tmp_parents = {}

        canbreak = True # assume go to rand and can break!
        new_node = [0, 0, 0, 0, 0, 0]
        near_node = deepcopy(node1)

                    
        for i in range(len(joint_limits)):
            if abs(node2[i] - near_node[i]) <= step_size:
                new_node[i] = deepcopy(node2[i])
            else:
                new_node[i] = near_node[i] + dirs[i]*step_size
                canbreak = False
        # near_node2 = [0, 0, 0, 0, 0, 0]
        if(canbreak):
            tmp_parents[tuple(node2)] = node1
            for key in tmp_parents:
                if key == tmp_parents[key] :
                    print("node 1 to node 2")
                    print("key: ", rounded_tuple(key))
                    print("val : ",rounded_tuple(tmp_parents[key]))
                    print("Deadlock here!")

                parents[key] = tmp_parents[key] # copy and paste only if success!
            return True, new_nodes, parents

        near_node2 = deepcopy(near_node)
        while not collision(new_node) and in_limit(new_node) and not canbreak:
            new_nodes.append(tuple(new_node))
            tmp_parents[tuple(new_node)] = tuple(near_node)
            if new_node == near_node:
                print("Deadlock due to here!")
            # parents[tuple(new_node)] = tuple(near_node)
            near_node = deepcopy(tuple(new_node)) # update parent
            canbreak = True
            # update new_node
            for i in range(len(joint_limits)):
                if abs(node2[i] - near_node[i]) <= step_size:
                    new_node[i] = node2[i]       

                else:
                    new_node[i] = near_node[i] + dirs[i]*step_size
                    canbreak = False
            near_node2 = deepcopy(near_node) ## KEY!! Why need another variable? use coy!

        # Problem here: Deadlock on the 'last node' : set parent[node2] to be near
        if canbreak:
            new_nodes.append(tuple(node2))
            # print("len of tmp parents: ",len(tmp_parents))
            lastkey = node1
            for key in tmp_parents:
                # print("key: ", rounded_tuple(key))
                # print("val : ",rounded_tuple(tmp_parents[key]))
                if key == tmp_parents[key] :
                    tmp_parents[key] = lastkey
                # Successful shortcut! Add tmp_parents {key:value} to parents
                parents[key] = tmp_parents[key] # copy and paste only if success!
                lastkey = deepcopy(key)
            return True, new_nodes, parents
        else:
            # Fail, don't do any modification XD
            return False, new_nodes, parents


    ITERATIONS = 150 # should be 150
    for it in range(ITERATIONS):
        id1, id2 = np.random.randint(low=0, high =len(path)-1, size=2)
        while(id1 == id2): # KEY : CANNOT Pick two equal points! Otherwise: Deadlock!!
            id1, id2 = np.random.randint(low=1, high=len(path)-2, size=2)
        if id1 < id2:
            n1 = path[id1]
            n2 = path[id2]
        else:
            n1 = path[id2]
            n2 = path[id1]
        success, new_nodes, parents =  try_shortcut(parents, n1, n2, step_size)

    print("Extracing optimized path...")
    optimized_path = extract_path(parents, final_node, root_parent, False) #debug=True)

    print("\n =======================")
    print("First Path    : ", len(path)," nodes")
    print("Optimized Path: ", len(optimized_path), " nodes")


    print("Now execute optimized_path:")

    print("Optimized path:")
    bluecolor = (0, 0, 1, 1)
    for pose in optimized_path:
        print(rounded_tuple(pose))
        set_joint_positions(PR2, joint_idx, pose)
        ee_pose = get_link_pose(PR2, link_from_name(PR2, 'l_gripper_tool_frame'))
        draw_sphere_marker(ee_pose[0], radius, bluecolor)
    ######################
    
    # Execute planned path
    print("Now executing shortcut smoothed path: ")    
    execute_trajectory(robots['pr2'], joint_idx, optimized_path, sleep=0.1)
    # Keep graphics window opened
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()


'''
TODO : Shortcut Smoothing
Idea: 
1. Randomly pick two nodes in rrt (a python list)
2. use rrt-connect to try connect two node
3. if connect success: (node obstacles in between):
       change parent[b] = a

'''
