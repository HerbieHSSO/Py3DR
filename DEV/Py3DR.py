import numpy as np
import cv2 as cv
import open3d as o3d

def write_ply(fn, verts, colors, verts2=False, colors2=False):
    ply = '''ply
    format ascii 1.0
    element vertex %(vert_num)d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    '''
    verts = verts.reshape(-1, 3)
    colors = colors.reshape(-1, 3)
    verts = np.hstack([verts, colors])

    verts2 = verts2.reshape(-1, 3)
    colors2 = colors2.reshape(-1, 3)
    verts2 = np.hstack([verts2, colors2])

    verts3 = np.concatenate((verts,  verts2), axis=0)
    try:
        with open(fn, 'wb') as f:
            f.write((ply % dict(vert_num=len(verts3))).encode('utf-8'))
            np.savetxt(f, verts3, fmt='%f %f %f %d %d %d ')
    except:
        with open(fn, 'wb') as f:
            f.write((ply % dict(vert_num=len(verts3))).encode('utf-8'))
            np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')
def read_ply(ply_file):
    read_ply_file = o3d.io.read_point_cloud(ply_file)
    o3d.visualization.draw_geometries([read_ply_file])
    
def reconstruction3D(imgLeft, imgRight, imgFront=False, imgBack=False, out_ply):
    #Downscale image Left
    imgL = cv.pyrDown(cv.imread(cv.samples.findFile(imgLeft)))
    #Downscale image Right
    imgR = cv.pyrDown(cv.imread(cv.samples.findFile(imgRight))) 


    #Set stereo data
    window_size = 3
    min_disparity = 16
    num_disparities = 112 - min_disparity
    stereo = cv.StereoSGBM_create(minDisparity = min_disparity,
        numDisparities = num_disparities,
        blockSize = 16,
        P1 = 8*3*window_size**2,
        P2 = 32*3*window_size**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    print('computing disparity...')
    disparity = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    print('generating 3d point cloud...',)
    h, w = imgL.shape[:2]
    f = 0.8*w                          # guess for focal length
    Q = np.float32([[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1,      0]])
    
    points_3D = cv.reprojectImageTo3D(disparity, Q)
    colors = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
    mask = disparity > disparity.min()
    out_points_3D = points_3D[mask]
    out_colors = colors[mask]





    #Downscale image Left
    imgF = cv.pyrDown(cv.imread(cv.samples.findFile(imgFront)))
    #Downscale image Right
    imgB = cv.pyrDown(cv.imread(cv.samples.findFile(imgBack))) 


    #Set stereo data
    window_size2 = 3
    min_disparity2 = 16
    num_disparities2 = 112 - min_disparity2
    stereo2 = cv.StereoSGBM_create(minDisparity = min_disparity2,
        numDisparities = num_disparities2,
        blockSize = 16,
        P1 = 8*3*window_size2**2,
        P2 = 32*3*window_size2**2,
        disp12MaxDiff = 1,
        uniquenessRatio = 10,
        speckleWindowSize = 100,
        speckleRange = 32
    )

    print('computing disparity...')
    disparity2 = stereo.compute(imgF, imgB).astype(np.float32) / 16.0

    print('generating 3d point cloud...',)
    h2, w2 = imgF.shape[:2]
    f2 = 0.8*w2                          # guess for focal length
    Q2 = np.float32([[1, 0, 0, -0.5*w2],
                    [0,-1, 0,  0.5*h2], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f2], # so that y-axis looks up
                    [0, 0, 1,      0]])
    
    points_3D2 = cv.reprojectImageTo3D(disparity2, Q2)
    colors2 = cv.cvtColor(imgF, cv.COLOR_BGR2RGB)
    mask2 = disparity2 > disparity2.min()
    out_points_3D2 = points_3D2[mask]
    out_colors2 = colors2[mask]


    write_ply(out_ply, out_points_3D, out_colors, out_points_3D2=False, out_colors2=False)
    print('%s saved' % out_ply)



