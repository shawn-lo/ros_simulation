import pcl
import numpy as np
import cv2
import math
def construct_vector_image(coords):
    h, w = coords.shape[:2]
    hmap = np.zeros((h-2,w-2,3))
    vmap = np.zeros((h-2,w-2,3))
    for i in range(1, w-1):
        for j in range(1, h-1):
            hmap[j-1][i-1] = np.subtract(coords[j][i+1], coords[j][i-1])
            vmap[j-1][i-1] = np.subtract(coords[j+1][i], coords[j-1][i])
    return hmap, vmap

def construct_integral_image( coords):
    hmap, vmap = construct_vector_image(coords)
    h, w = hmap.shape[:2]
    ihmap = np.zeros((h, w, 3))
    ivmap = np.zeros((h, w, 3))
    
    ihmap[0][0] = hmap[0][0]
    ivmap[0][0] = vmap[0][0]
    # first row
    for i in range(1,w):
        ihmap[0][i] = np.add(ihmap[0][i-1], hmap[0][i])
        ivmap[0][i] = np.add(ivmap[0][i-1], vmap[0][i])
    # first col
    for j in range(1, h):
        ihmap[j][0] = np.add(ihmap[j-1][0], hmap[j][0])
        ivmap[j][0] = np.add(ivmap[j-1][0], vmap[j][0])
    # rest
    for i in range(1, w):
        for j in range(1, h):
            ihmap[j][i] = ihmap[j-1][i] + ihmap[j][i-1] - ihmap[j-1][i-1] + hmap[j][i]
            ivmap[j][i] = ivmap[j-1][i] + ivmap[j][i-1] - ivmap[j-1][i-1] + vmap[j][i]

    return ihmap, ivmap

def get_point_cloud(depth_map, focal=554.254691191187):
    h, w = depth_map.shape[:2]
    res = list()
    coords = np.zeros((h,w,3))
    for i in range(w):
        for j in range(h):
            if(math.isnan(depth_map[j][i])):
                coords[j,i,:] = (0,0,0)
                res.append((float('nan'), float('nan'), float('nan')))
            else:
                x = abs(w/2) - i
                y = abs(h/2) - j
                depth = depth_map[j][i]
                cX = x*depth/focal
                cY = y*depth/focal
                coords[j,i,:] = (cX, cY, depth) 
                res.append((cX, cY, depth))
    return res, coords

def cal_normal(depth_map, smooth='off', radius=3):
    pc_list, coords = get_point_cloud(depth_map)
    p = pcl.PointCloud()
    p.from_list(pc_list)
    pcl.save(p, 'test.pcd')
    if smooth == 'on':
        ihmap, ivmap = construct_integral_image(coords)
        h, w = ihmap.shape[:2]
        normal_map = np.zeros((h,w,3))
        for i in range(radius+1, w-radius):
            for j in range(radius+1, h-radius):
                havg = ihmap[j+radius][i+radius] + ihmap[j-radius-1][i-radius-1] - ihmap[j-radius-1][i+radius] - ihmap[j+radius][i-radius-1]
                vavg = ivmap[j+radius][i+radius] + ivmap[j-radius-1][i-radius-1] - ivmap[j-radius-1][i+radius] - ivmap[j+radius][i-radius-1]
                vector = np.cross(havg, vavg)
                normal_map[j][i] = vector/np.linalg.norm(vector)
    elif smooth == 'off':
        hmap, vmap = construct_vector_image(coords)
        h, w = hmap.shape[:2]
        normal_map = np.zeros((h,w,3))
        for i in range(0, w):
            for j in range(0, h):
                vector = np.cross(hmap[j][i], vmap[j][i])
                #if(j > 240):
                #    print(vector)
                normal_map[j][i] = vector/np.linalg.norm(vector)
    return normal_map

def ground_detection(normal_map):
    h, w = normal_map.shape[:2]
    res = np.zeros((h,w))
    tol = 0.01
    for i in range(w):
        for j in range(h):
            nx,ny,nz = normal_map[j][i]
            if abs(ny) > 1-tol:
                res[j][i] = 1
    return res

def is_obstacle(depth_map, ground_map):
    h, w = ground_map.shape[:2]
    print(h,w)
    res = np.zeros((h,w,3))
    for i in range(w):
        for j in range(h):
            if(depth_map[j][i] < 0.55):
                if ground_map[j][i] == 0:
                    res[j,i,:] = (0,255,0)
                else:
                    res[j,i,:] = (255,0,0)
    return res

