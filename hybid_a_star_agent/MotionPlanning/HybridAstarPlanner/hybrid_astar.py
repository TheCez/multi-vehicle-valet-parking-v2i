"""
Hybrid A*
@author: Huiming Zhou
"""

import os
import sys
import math
import time
from heapdict import heapdict
import numpy as np
import matplotlib.pyplot as plt
from scipy import spatial as kd
import io
from PIL import Image
import cv2
from matplotlib.patches import Rectangle
import threading

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../MotionPlanning/")

import HybridAstarPlanner.astar as astar
import HybridAstarPlanner.draw as draw
import CurvesGenerator.reeds_shepp as rs


class C:  # Parameter config
    PI = math.pi

    XY_RESO = 2  # [m]
    YAW_RESO = np.deg2rad(15.0)  # [rad]
    MOVE_STEP = 0.4  # [m] path interporate resolution
    N_STEER = 20.0  # steer command number
    COLLISION_CHECK_STEP = 5  # skip number for collision check
    EXTEND_BOUND = 1  # collision check range extended

    GEAR_COST = 100.0  # switch back penalty cost
    BACKWARD_COST = 5.0  # backward penalty cost
    STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
    STEER_ANGLE_COST = 1.0  # steer angle penalty cost
    H_COST = 15.0  # Heuristic cost penalty cost

    # RF = 4.5  # [m] distance from rear to vehicle front end of vehicle
    # RB = 1.0  # [m] distance from rear to vehicle back end of vehicle
    # W = 2.5  # [m] width of vehicle
    # WD = 0.7 * W  # [m] distance between left-right wheels
    # WB = 3.5  # [m] Wheel base
    # TR = 0.5  # [m] Tyre radius
    # TW = 1  # [m] Tyre width
    # MAX_STEER = 0.6  # [rad] maximum steering angle

    # RF =6.5  # [m] distance from rear to vehicle front end of vehicle
    # RB = 1.5  # [m] distance from rear to vehicle back end of vehicle
    # W = 5  # [m] width of vehicle
    # WD = 0.7 * W  # [m] distance between left-right wheels
    # WB = 5  # [m] Wheel base
    # TR = 0.55  # [m] Tyre radius
    # TW =  0.4 # [m] Tyre width
    # MAX_STEER = 1.13  # [rad] maximum steering angle

    RF =6.5  # [m] distance from rear to vehicle front end of vehicle
    RB = 1.5  # [m] distance from rear to vehicle back end of vehicle
    W = 5  # [m] width of vehicle
    WD = 0.7 * W  # [m] distance between left-right wheels
    WB = 5  # [m] Wheel base
    TR = 0.55  # [m] Tyre radius
    TW =  0.4 # [m] Tyre width
    MAX_STEER = 0.6  # [rad] maximum steering angle


class Node:
    def __init__(self, xind, yind, yawind, direction, x, y,
                 yaw, directions, steer, cost, pind):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.x = x
        self.y = y
        self.yaw = yaw
        self.directions = directions
        self.steer = steer
        self.cost = cost
        self.pind = pind


class Para:
    def __init__(self, minx, miny, minyaw, maxx, maxy, maxyaw,
                 xw, yw, yaww, xyreso, yawreso, ox, oy, kdtree):
        self.minx = minx
        self.miny = miny
        self.minyaw = minyaw
        self.maxx = maxx
        self.maxy = maxy
        self.maxyaw = maxyaw
        self.xw = xw
        self.yw = yw
        self.yaww = yaww
        self.xyreso = xyreso
        self.yawreso = yawreso
        self.ox = ox
        self.oy = oy
        self.kdtree = kdtree


class Path:
    def __init__(self, x, y, yaw, direction, cost):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction
        self.cost = cost


class QueuePrior:
    def __init__(self):
        self.queue = heapdict()

    def empty(self):
        return len(self.queue) == 0  # if Q is empty

    def put(self, item, priority):
        self.queue[item] = priority  # push 

    def get(self):
        return self.queue.popitem()[0]  # pop out element with smallest priority


def hybrid_astar_planning(sx, sy, syaw, gx, gy, gyaw, ox, oy, xyreso, yawreso):
    sxr, syr = round(sx / xyreso), round(sy / xyreso)
    gxr, gyr = round(gx / xyreso), round(gy / xyreso)
    syawr = round(rs.pi_2_pi(syaw) / yawreso)
    gyawr = round(rs.pi_2_pi(gyaw) / yawreso)

    nstart = Node(sxr, syr, syawr, 1, [sx], [sy], [syaw], [1], 0.0, 0.0, -1)
    ngoal = Node(gxr, gyr, gyawr, 1, [gx], [gy], [gyaw], [1], 0.0, 0.0, -1)

    kdtree = kd.KDTree([[x, y] for x, y in zip(ox, oy)])
    P = calc_parameters(ox, oy, xyreso, yawreso, kdtree)

    hmap = astar.calc_holonomic_heuristic_with_obstacle(ngoal, P.ox, P.oy, P.xyreso, 1.0)
    steer_set, direc_set = calc_motion_set()
    open_set, closed_set = {calc_index(nstart, P): nstart}, {}

    qp = QueuePrior()
    qp.put(calc_index(nstart, P), calc_hybrid_cost(nstart, hmap, P))

    while True:
        if not open_set:
            return None

        ind = qp.get()
        n_curr = open_set[ind]
        closed_set[ind] = n_curr
        open_set.pop(ind)

        update, fpath = update_node_with_analystic_expantion(n_curr, ngoal, P)

        if update:
            fnode = fpath
            break

        for i in range(len(steer_set)):
            node = calc_next_node(n_curr, ind, steer_set[i], direc_set[i], P)

            if not node:
                continue

            node_ind = calc_index(node, P)

            if node_ind in closed_set:
                continue

            if node_ind not in open_set:
                open_set[node_ind] = node
                qp.put(node_ind, calc_hybrid_cost(node, hmap, P))
            else:
                if open_set[node_ind].cost > node.cost:
                    open_set[node_ind] = node
                    qp.put(node_ind, calc_hybrid_cost(node, hmap, P))

    return extract_path(closed_set, fnode, nstart)


def extract_path(closed, ngoal, nstart):
    rx, ry, ryaw, direc = [], [], [], []
    cost = 0.0
    node = ngoal

    while True:
        rx += node.x[::-1]
        ry += node.y[::-1]
        ryaw += node.yaw[::-1]
        direc += node.directions[::-1]
        cost += node.cost

        if is_same_grid(node, nstart):
            break

        node = closed[node.pind]

    rx = rx[::-1]
    ry = ry[::-1]
    ryaw = ryaw[::-1]
    direc = direc[::-1]

    direc[0] = direc[1]
    path = Path(rx, ry, ryaw, direc, cost)

    return path


def calc_next_node(n_curr, c_id, u, d, P):
    step = C.XY_RESO * 2

    nlist = math.ceil(step / C.MOVE_STEP)
    xlist = [n_curr.x[-1] + d * C.MOVE_STEP * math.cos(n_curr.yaw[-1])]
    ylist = [n_curr.y[-1] + d * C.MOVE_STEP * math.sin(n_curr.yaw[-1])]
    yawlist = [rs.pi_2_pi(n_curr.yaw[-1] + d * C.MOVE_STEP / C.WB * math.tan(u))]

    for i in range(nlist - 1):
        xlist.append(xlist[i] + d * C.MOVE_STEP * math.cos(yawlist[i]))
        ylist.append(ylist[i] + d * C.MOVE_STEP * math.sin(yawlist[i]))
        yawlist.append(rs.pi_2_pi(yawlist[i] + d * C.MOVE_STEP / C.WB * math.tan(u)))

    xind = round(xlist[-1] / P.xyreso)
    yind = round(ylist[-1] / P.xyreso)
    yawind = round(yawlist[-1] / P.yawreso)

    if not is_index_ok(xind, yind, xlist, ylist, yawlist, P):
        return None

    cost = 0.0

    if d > 0:
        direction = 1
        cost += abs(step)
    else:
        direction = -1
        cost += abs(step) * C.BACKWARD_COST

    if direction != n_curr.direction:  # switch back penalty
        cost += C.GEAR_COST

    cost += C.STEER_ANGLE_COST * abs(u)  # steer angle penalyty
    cost += C.STEER_CHANGE_COST * abs(n_curr.steer - u)  # steer change penalty
    cost = n_curr.cost + cost

    directions = [direction for _ in range(len(xlist))]

    node = Node(xind, yind, yawind, direction, xlist, ylist,
                yawlist, directions, u, cost, c_id)

    return node


def is_index_ok(xind, yind, xlist, ylist, yawlist, P):
    if xind <= P.minx or \
            xind >= P.maxx or \
            yind <= P.miny or \
            yind >= P.maxy:
        return False

    ind = range(0, len(xlist), C.COLLISION_CHECK_STEP)

    nodex = [xlist[k] for k in ind]
    nodey = [ylist[k] for k in ind]
    nodeyaw = [yawlist[k] for k in ind]

    if is_collision(nodex, nodey, nodeyaw, P):
        return False

    return True


def update_node_with_analystic_expantion(n_curr, ngoal, P):
    path = analystic_expantion(n_curr, ngoal, P)  # rs path: n -> ngoal

    if not path:
        return False, None

    fx = path.x[1:-1]
    fy = path.y[1:-1]
    fyaw = path.yaw[1:-1]
    fd = path.directions[1:-1]

    fcost = n_curr.cost + calc_rs_path_cost(path)
    fpind = calc_index(n_curr, P)
    fsteer = 0.0

    fpath = Node(n_curr.xind, n_curr.yind, n_curr.yawind, n_curr.direction,
                 fx, fy, fyaw, fd, fsteer, fcost, fpind)

    return True, fpath


def analystic_expantion(node, ngoal, P):
    sx, sy, syaw = node.x[-1], node.y[-1], node.yaw[-1]
    gx, gy, gyaw = ngoal.x[-1], ngoal.y[-1], ngoal.yaw[-1]

    maxc = math.tan(C.MAX_STEER) / C.WB
    paths = rs.calc_all_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size=C.MOVE_STEP)

    if not paths:
        return None

    pq = QueuePrior()
    for path in paths:
        pq.put(path, calc_rs_path_cost(path))

    while not pq.empty():
        path = pq.get()
        ind = range(0, len(path.x), C.COLLISION_CHECK_STEP)

        pathx = [path.x[k] for k in ind]
        pathy = [path.y[k] for k in ind]
        pathyaw = [path.yaw[k] for k in ind]

        if not is_collision(pathx, pathy, pathyaw, P):
            return path

    return None


def is_collision(x, y, yaw, P):
    for ix, iy, iyaw in zip(x, y, yaw):
        d = 1
        dl = (C.RF - C.RB) / 2.0
        r = (C.RF + C.RB) / 2.0 + d

        cx = ix + dl * math.cos(iyaw)
        cy = iy + dl * math.sin(iyaw)

        ids = P.kdtree.query_ball_point([cx, cy], r)

        if not ids:
            continue

        for i in ids:
            xo = P.ox[i] - cx
            yo = P.oy[i] - cy
            dx = xo * math.cos(iyaw) + yo * math.sin(iyaw)
            dy = -xo * math.sin(iyaw) + yo * math.cos(iyaw)

            if abs(dx) < r and abs(dy) < C.W / 2 + d:
                return True

    return False


def calc_rs_path_cost(rspath):
    cost = 0.0

    for lr in rspath.lengths:
        if lr >= 0:
            cost += 1
        else:
            cost += abs(lr) * C.BACKWARD_COST

    for i in range(len(rspath.lengths) - 1):
        if rspath.lengths[i] * rspath.lengths[i + 1] < 0.0:
            cost += C.GEAR_COST

    for ctype in rspath.ctypes:
        if ctype != "S":
            cost += C.STEER_ANGLE_COST * abs(C.MAX_STEER)

    nctypes = len(rspath.ctypes)
    ulist = [0.0 for _ in range(nctypes)]

    for i in range(nctypes):
        if rspath.ctypes[i] == "R":
            ulist[i] = -C.MAX_STEER
        elif rspath.ctypes[i] == "WB":
            ulist[i] = C.MAX_STEER

    for i in range(nctypes - 1):
        cost += C.STEER_CHANGE_COST * abs(ulist[i + 1] - ulist[i])

    return cost


def calc_hybrid_cost(node, hmap, P):
    cost = node.cost + \
           C.H_COST * hmap[node.xind - P.minx][node.yind - P.miny]

    return cost


def calc_motion_set():
    s = np.arange(C.MAX_STEER / C.N_STEER,
                  C.MAX_STEER, C.MAX_STEER / C.N_STEER)

    steer = list(s) + [0.0] + list(-s)
    direc = [1.0 for _ in range(len(steer))] + [-1.0 for _ in range(len(steer))]
    steer = steer + steer

    return steer, direc


def is_same_grid(node1, node2):
    if node1.xind != node2.xind or \
            node1.yind != node2.yind or \
            node1.yawind != node2.yawind:
        return False

    return True


def calc_index(node, P):
    ind = (node.yawind - P.minyaw) * P.xw * P.yw + \
          (node.yind - P.miny) * P.xw + \
          (node.xind - P.minx)

    return ind


def calc_parameters(ox, oy, xyreso, yawreso, kdtree):
    minx = round(min(ox) / xyreso)
    miny = round(min(oy) / xyreso)
    maxx = round(max(ox) / xyreso)
    maxy = round(max(oy) / xyreso)

    xw, yw = maxx - minx, maxy - miny

    minyaw = round(-C.PI / yawreso) - 1
    maxyaw = round(C.PI / yawreso)
    yaww = maxyaw - minyaw

    return Para(minx, miny, minyaw, maxx, maxy, maxyaw,
                xw, yw, yaww, xyreso, yawreso, ox, oy, kdtree)


def draw_car(x, y, yaw, steer, color='black'):
    car = np.array([[-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                    [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2]])

    wheel = np.array([[-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                      [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4]])

    rlWheel = wheel.copy()
    rrWheel = wheel.copy()
    frWheel = wheel.copy()
    flWheel = wheel.copy()

    Rot1 = np.array([[math.cos(yaw), -math.sin(yaw)],
                     [math.sin(yaw), math.cos(yaw)]])

    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    frWheel = np.dot(Rot2, frWheel)
    flWheel = np.dot(Rot2, flWheel)

    frWheel += np.array([[C.WB], [-C.WD / 2]])
    flWheel += np.array([[C.WB], [C.WD / 2]])
    rrWheel[1, :] -= C.WD / 2
    rlWheel[1, :] += C.WD / 2

    frWheel = np.dot(Rot1, frWheel)
    flWheel = np.dot(Rot1, flWheel)

    rrWheel = np.dot(Rot1, rrWheel)
    rlWheel = np.dot(Rot1, rlWheel)
    car = np.dot(Rot1, car)

    frWheel += np.array([[x], [y]])
    flWheel += np.array([[x], [y]])
    rrWheel += np.array([[x], [y]])
    rlWheel += np.array([[x], [y]])
    car += np.array([[x], [y]])

    plt.plot(car[0, :], car[1, :], color)
    plt.plot(frWheel[0, :], frWheel[1, :], color)
    plt.plot(rrWheel[0, :], rrWheel[1, :], color)
    plt.plot(flWheel[0, :], flWheel[1, :], color)
    plt.plot(rlWheel[0, :], rlWheel[1, :], color)
    draw.Arrow(x, y, yaw, C.WB * 0.8, color)


def convert_grid_to_obstacles(grid_file):
    # Load the existing 2D grid
    grid = grid_file

    # Get grid dimensions
    y, x = grid.shape  # Note: numpy arrays are indexed as [row, column]

    ox, oy = [], []

    # Find obstacle coordinates
    for i in range(x):
        for j in range(y):
            if grid[j, i] == 1:  # If it's an obstacle
                ox.append(i)
                oy.append(j)

    return ox, oy

def convert_grid_to_obstacles_instance_segmentation(grid_file):
    grid = np.load(grid_file)
    # Apply thinning
    kernel = np.ones((3,3), np.uint8)
    thinned = cv2.erode(grid.astype(np.uint8), kernel, iterations=1)
    
    ox, oy = [], []
    y, x = thinned.shape
    for i in range(x):
        for j in range(y):
            if thinned[j, i] == 1:
                ox.append(i)
                oy.append(j)
    return ox, oy



def design_obstacles(x, y):
    ox, oy = [], []

    #lower bound
    for i in range(x):
        ox.append(i)
        oy.append(0)
    #upper bound
    for i in range(x):
        ox.append(i)
        oy.append(y - 1)
    #left bound
    for i in range(y):
        ox.append(0)
        oy.append(i)
    #right bound
    for i in range(y):
        ox.append(x - 1)
        oy.append(i)

    #obstacles
    for i in range (0, 51,5):
        for j in range(0, 8):
            ox.append(i)
            oy.append(j)

    for i in range (0, 51,5):
        for j in range(23, 30):
            ox.append(i)
            oy.append(j)
    
    # for i in range(0, 10):
    #     ox.append(7)
    #     oy.append(i)
    
    #obstacles
    # for i in range(10, 21):
    #     ox.append(i)
    #     oy.append(15)
    # for i in range(15):
    #     ox.append(20)
    #     oy.append(i)
    # for i in range(15, 30):
    #     ox.append(30)
    #     oy.append(i)
    # for i in range(16):
    #     ox.append(40)
    #     oy.append(i)

    return ox, oy

def plot_map(ox, oy):
    
    plt.cla()
    plt.plot(ox, oy, "sk")
    plt.axis("equal")
    # plt.plot(x, y, linewidth=1.5, color='r')
    plt.show()
    print("Done!")

def draw_blue_box(ax, x, y, yaw, width=10, height=10):
    box = Rectangle((-width/2, -height/2), width, height, 
                    facecolor='blue', 
                    edgecolor='blue', 
                    alpha=0.5)
    t = plt.matplotlib.transforms.Affine2D().rotate(yaw)
    t = t.translate(x, y)
    t += ax.transData
    box.set_transform(t)
    ax.add_patch(box)
    ax.plot(x, y, 'go', markersize=6, markeredgecolor='darkgreen', markerfacecolor='limegreen')

def world_to_grid(x, y, center, cell_size):
    gx = int(center + x / cell_size)
    gy = int(center + y / cell_size)
    return gx, gy

def path_finder(sx, sy, syaw0, gx, gy, gyaw0, occupation_grid):
    print("start!")
    #x, y = 500, 500
    # Convert CARLA world coordinates to grid coordinates
    # Assumptions: 
    # - occupation_grid is a path to the .npy file
    # - grid is square (grid_size x grid_size)
    # - center is at grid_size // 2
    # - cell_size is known (meters per cell)
    grid = occupation_grid
    grid_size = grid.shape[0]
    center = grid_size // 2
    cell_size = 0.5  # <-- set this to your cell size in meters

  

    sx, sy = world_to_grid(sx, sy, center, cell_size)
    gx, gy = world_to_grid(gx, gy, center, cell_size)
    # Yaw does not need conversion if it's in radians and matches grid orientation
    ox, oy = convert_grid_to_obstacles(occupation_grid)
    path = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0,
                                 ox, oy, C.XY_RESO, C.YAW_RESO)
    
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    # frames = []

    # for k in range(len(x)):
    #     plt.cla()
    #     plt.plot(ox, oy, "sk")
    #     plt.plot(x, y, linewidth=1, color='r')
    #     #plt.gca().invert_xaxis()
    #     plt.gca().invert_yaxis()

    #     # Get current axis for drawing blue box
    #     ax = plt.gca()
        
    #     # Draw blue box at current position with current yaw
    #     draw_blue_box(ax, x[k], y[k], yaw[k])

    #     if k < len(x) - 2:
    #         dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
    #         steer = rs.pi_2_pi(math.atan(-C.WB * dy / direction[k]))
    #     else:
    #         steer = 0.0

    #     draw_car(gx, gy, gyaw0, 0.0, 'dimgray')
    #     draw_car(x[k], y[k], yaw[k], steer)
    #     plt.title("Hybrid A*")
    #     plt.axis("equal")
    #     plt.pause(0.0001)
    #     buf = io.BytesIO()
    #     plt.savefig(buf, format='png')
    #     buf.seek(0)
    #     frames.append(Image.open(buf))

    # plt.show()
    print("Done!")
    return path

def path_finder2(sx, sy, syaw0, gx, gy, gyaw0, occupation_grid):
    print("start!")
    #x, y = 500, 500
    # Convert CARLA world coordinates to grid coordinates
    # Assumptions: 
    # - occupation_grid is a path to the .npy file
    # - grid is square (grid_size x grid_size)
    # - center is at grid_size // 2
    # - cell_size is known (meters per cell)
    grid = occupation_grid
    grid_size = grid.shape[0]
    center = grid_size // 2
    cell_size = 0.5  # <-- set this to your cell size in meters

  

    sx, sy = world_to_grid(sx, sy, center, cell_size)
    # gx, gy = world_to_grid(gx, gy, center, cell_size)
    # Yaw does not need conversion if it's in radians and matches grid orientation
    ox, oy = convert_grid_to_obstacles(occupation_grid)
    path = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0,
                                 ox, oy, C.XY_RESO, C.YAW_RESO)
    
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    # frames = []

    # for k in range(len(x)):
    #     plt.cla()
    #     plt.plot(ox, oy, "sk")
    #     plt.plot(x, y, linewidth=1, color='r')
    #     #plt.gca().invert_xaxis()
    #     plt.gca().invert_yaxis()

    #     # Get current axis for drawing blue box
    #     ax = plt.gca()
        
    #     # Draw blue box at current position with current yaw
    #     draw_blue_box(ax, x[k], y[k], yaw[k])

    #     if k < len(x) - 2:
    #         dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
    #         steer = rs.pi_2_pi(math.atan(-C.WB * dy / direction[k]))
    #     else:
    #         steer = 0.0

    #     draw_car(gx, gy, gyaw0, 0.0, 'dimgray')
    #     draw_car(x[k], y[k], yaw[k], steer)
    #     plt.title("Hybrid A*")
    #     plt.axis("equal")
    #     plt.pause(0.0001)
    #     buf = io.BytesIO()
    #     plt.savefig(buf, format='png')
    #     buf.seek(0)
    #     frames.append(Image.open(buf))

    # plt.show()
    print("Done!")
    return path

def short_path_finder(sx, sy, syaw0, gx, gy, gyaw0, occupation_grid, min_x, min_y, pad_x, pad_y):
    print("start!")
    #x, y = 500, 500
    # Convert CARLA world coordinates to grid coordinates
    # Assumptions: 
    # - occupation_grid is a path to the .npy file
    # - grid is square (grid_size x grid_size)
    # - center is at grid_size // 2
    # - cell_size is known (meters per cell)
    grid = occupation_grid
    grid_size = 500  # Assuming a 500x500 grid
    center = grid_size // 2
    cell_size = 0.5  # <-- set this to your cell size in meters

  

    sx, sy = world_to_grid(sx, sy, center, cell_size)
    sx = sx - min_x + pad_x
    sy = sy - min_y + pad_y
    gx = gx + pad_x
    gy = gy + pad_y

    print(f"sx: {sx}, sy: {sy}, gx: {gx}, gy: {gy}")
    #gx, gy = world_to_grid(gx, gy, center, cell_size)
    # Yaw does not need conversion if it's in radians and matches grid orientation
    ox, oy = convert_grid_to_obstacles(occupation_grid)
    path = None
    # def run_planner():
    #     nonlocal path
    #     result = [None]
    #     def planner():
    #         result[0] = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0, ox, oy, C.XY_RESO, C.YAW_RESO)
    #     planner_thread = threading.Thread(target=planner)
    #     planner_thread.start()
    #     planner_thread.join(timeout=1.5)
    #     if planner_thread.is_alive():
    #         print("Planning timed out!")
    #         path = None
    #     else:
    #         path = result[0]

    # run_planner()


    path = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0,
                                     ox, oy, C.XY_RESO, C.YAW_RESO)
    
    if path is None:
        print("No path found!")
        return None
    
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    # frames = []

    # for k in range(len(x)):
    #     plt.cla()
    #     plt.plot(ox, oy, "sk")
    #     plt.plot(x, y, linewidth=1, color='r')
    #     #plt.gca().invert_xaxis()
    #     plt.gca().invert_yaxis()

    #     # Get current axis for drawing blue box
    #     ax = plt.gca()
        
    #     # Draw blue box at current position with current yaw
    #     draw_blue_box(ax, x[k], y[k], yaw[k])

    #     if k < len(x) - 2:
    #         dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
    #         steer = rs.pi_2_pi(math.atan(-C.WB * dy / direction[k]))
    #     else:
    #         steer = 0.0

    #     draw_car(gx, gy, gyaw0, 0.0, 'dimgray')
    #     draw_car(x[k], y[k], yaw[k], steer)
    #     plt.title("Hybrid A*")
    #     plt.axis("equal")
    #     plt.pause(0.0001)
    #     buf = io.BytesIO()
    #     plt.savefig(buf, format='png')
    #     buf.seek(0)
    #     frames.append(Image.open(buf))

    # plt.show()
    print("Done!")
    return path


def short_path_finder2(sx, sy, syaw0, gx, gy, gyaw0, occupation_grid, min_x, min_y, pad_x, pad_y):
    print("start!")
    #x, y = 500, 500
    # Convert CARLA world coordinates to grid coordinates
    # Assumptions: 
    # - occupation_grid is a path to the .npy file
    # - grid is square (grid_size x grid_size)
    # - center is at grid_size // 2
    # - cell_size is known (meters per cell)
    grid = occupation_grid
    grid_size = 500  # Assuming a 500x500 grid
    center = grid_size // 2
    cell_size = 0.5  # <-- set this to your cell size in meters

  

    #sx, sy = world_to_grid(sx, sy, center, cell_size)
    sx = sx - min_x + pad_x
    sy = sy - min_y + pad_y
    gx = gx - min_x + pad_x
    gy = gy - min_y + pad_y

    print(f"sx: {sx}, sy: {sy}, gx: {gx}, gy: {gy}")
    #gx, gy = world_to_grid(gx, gy, center, cell_size)
    # Yaw does not need conversion if it's in radians and matches grid orientation
    ox, oy = convert_grid_to_obstacles(occupation_grid)
    path = None
    # def run_planner():
    #     nonlocal path
    #     result = [None]
    #     def planner():
    #         result[0] = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0, ox, oy, C.XY_RESO, C.YAW_RESO)
    #     planner_thread = threading.Thread(target=planner)
    #     planner_thread.start()
    #     planner_thread.join(timeout=1.5)
    #     if planner_thread.is_alive():
    #         print("Planning timed out!")
    #         path = None
    #     else:
    #         path = result[0]

    # run_planner()


    path = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0,
                                     ox, oy, C.XY_RESO, C.YAW_RESO)
    
    if path is None:
        print("No path found!")
        return None
    
    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    # frames = []

    # for k in range(len(x)):
    #     plt.cla()
    #     plt.plot(ox, oy, "sk")
    #     plt.plot(x, y, linewidth=1, color='r')
    #     #plt.gca().invert_xaxis()
    #     plt.gca().invert_yaxis()

    #     # Get current axis for drawing blue box
    #     ax = plt.gca()
        
    #     # Draw blue box at current position with current yaw
    #     draw_blue_box(ax, x[k], y[k], yaw[k])

    #     if k < len(x) - 2:
    #         dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
    #         steer = rs.pi_2_pi(math.atan(-C.WB * dy / direction[k]))
    #     else:
    #         steer = 0.0

    #     draw_car(gx, gy, gyaw0, 0.0, 'dimgray')
    #     draw_car(x[k], y[k], yaw[k], steer)
    #     plt.title("Hybrid A*")
    #     plt.axis("equal")
    #     plt.pause(0.0001)
    #     buf = io.BytesIO()
    #     plt.savefig(buf, format='png')
    #     buf.seek(0)
    #     frames.append(Image.open(buf))

    # plt.show()
    print("Done!")
    return path


def main():
    print("start!")
    # x, y = 51, 31
    # x, y = 51, 31
    # sx, sy, syaw0 = 7.5, 6.0, np.deg2rad(90.0)
    # gx, gy, gyaw0 = 42.5, 24.0, np.deg2rad(90.0)
    #############################################################
    x, y = 500, 500
    sx, sy, syaw0 = 245, 327, np.deg2rad(270.0)
    # sx, sy, syaw0 = 245, 327, np.deg2rad(90.0)
    # gx, gy, gyaw0 = 245, 256, np.deg2rad(90.0)
    # gx, gy, gyaw0 = 270, 275, np.deg2rad(180.0)
    # sx, sy, syaw0 = 255, 275, np.deg2rad(180.0)
    # gx, gy, gyaw0 = 270, 300, np.deg2rad(270.0)
    # gx, gy, gyaw0 = 279, 310, np.deg2rad(180.0)
    gx, gy, gyaw0 = 275, 310, np.deg2rad(0.0)
    #############################################################

    # x, y = 800, 800
    # sx, sy, syaw0 = 10, 0, np.deg2rad(270.0)
    # # gx, gy, gyaw0 = 245, 256, np.deg2rad(90.0)
    # # gx, gy, gyaw0 = 270, 275, np.deg2rad(180.0)
    # # sx, sy, syaw0 = 255, 275, np.deg2rad(180.0)
    # # gx, gy, gyaw0 = 270, 300, np.deg2rad(270.0)
    # gx, gy, gyaw0 = 10, 50, np.deg2rad(270.0)
    # ox, oy = design_obstacles(x, y)

    # ox, oy = np.load('valet_map/grid.npy')

    # Load the .npy file
    # grid_file = 'valet_map/grid.npy'
    grid_file = 'valet_map/final_grid.npy'
    ox, oy = convert_grid_to_obstacles(grid_file)

    # print(grid.shape)

    # converted_array = np.any(grid > 0, axis=2).astype(int)

    # print(converted_array.shape)

    # Extract x and y coordinates
    # ox, oy = grid[:, 1], grid[:, 2]

    # plot_map(x, y, ox, oy)

    plot_map(ox, oy)

    t0 = time.time()
    path = None
    def run_planner():
        path = hybrid_astar_planning(sx, sy, syaw0, gx, gy, gyaw0,
                                        ox, oy, C.XY_RESO, C.YAW_RESO)

    planner_thread = threading.Thread(target=run_planner)
    planner_thread.start()
    planner_thread.join(timeout=2)
    if planner_thread.is_alive():
        print("Planning timed out!")
        path = None
    else:
        path = path
    if path is None:
        print("No path found!")
        return None
    print("Path length: ", (path.direction))
    # path = hybrid_astar_planning(2.5, 2.5, syaw0, 3, 25.5, np.deg2rad(60.0),
    #                              ox, oy, C.XY_RESO, C.YAW_RESO)
    # path = hybrid_astar_planning(2.5, 2.5, syaw0, 46.5, 6, gyaw0,
    #                              ox, oy, C.XY_RESO, C.YAW_RESO)
    t1 = time.time()
    print("running T: ", t1 - t0)

    if not path:
        print("Searching failed!")
        return

    x = path.x
    y = path.y
    yaw = path.yaw
    direction = path.direction
    frames = []

    for k in range(len(x)):
        plt.cla()
        plt.plot(ox, oy, "sk")
        plt.plot(x, y, linewidth=1.5, color='r')
        plt.gca().invert_xaxis()
        # Get current axis for drawing blue box
        ax = plt.gca()
        
        # Draw blue box at current position with current yaw
        draw_blue_box(ax, x[k], y[k], yaw[k])

        if k < len(x) - 2:
            dy = (yaw[k + 1] - yaw[k]) / C.MOVE_STEP
            steer = rs.pi_2_pi(math.atan(-C.WB * dy / direction[k]))
        else:
            steer = 0.0

        draw_car(gx, gy, gyaw0, 0.0, 'dimgray')
        draw_car(x[k], y[k], yaw[k], steer)
        plt.title("Hybrid A*")
        plt.axis("equal")
        plt.pause(0.0001)
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        frames.append(Image.open(buf))

    plt.show()
    print("Done!")

    # # Save frames as an animated GIF using Pillow
    # frames[0].save(
    #     'parking_lot3.gif',
    #     save_all=True,
    #     append_images=frames[1:],  # Add all frames except the first one
    #     duration=200,  # Duration of each frame in milliseconds
    #     loop=0  # Loop indefinitely (0 means infinite loop)
    # )


if __name__ == '__main__':
    main()
