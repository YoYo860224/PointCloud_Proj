import os
import math
import natsort
import numpy as np
from pypcd import pypcd

from sensor_msgs import msg
from sensor_msgs.point_cloud2 import PointCloud2, PointField

kittiRoot = "/media/yoyo/harddisk/kitti_odmetry/2011_09_26/2011_09_26_drive_0056_sync/"


def EulerToRotationMatrix(theta):
    R_x = np.array([[1,                   0,                   0],
                    [0,  math.cos(theta[0]),  math.sin(theta[0])],
                    [0, -math.sin(theta[0]),  math.cos(theta[0])]])

    R_y = np.array([[ math.cos(theta[1]), 0, math.sin(theta[1])],
                    [                  0, 1,                  0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])

    R_z = np.array([[ math.cos(theta[2]), math.sin(theta[2]), 0],
                    [-math.sin(theta[2]), math.cos(theta[2]), 0],
                    [                  0,                  0, 1]])
    
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R

def RotationMatrixToEuler(R):
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    # [x, y, z] = [roll, pitch, yaw]
    return np.array([x, y, z], dtype=np.float32)

def PCTransform(pc, offset, theta):
    pc_xyz = pc[:, 0:3]
    pc_ins = pc[:, 3:4]
    R = EulerToRotationMatrix(theta)
    pc_T = pc_xyz.dot(R) + offset
    pc_T = np.hstack((pc_T, pc_ins))
    return pc_T

def ReadGTPos(poseTXT):
    with open(poseTXT) as f:
        poses = np.array([[float(x) for x in line.split()] for line in f])
        gtPosition = poses[:, 12:15]

        R = np.array([[[x[0], x[4], x[8]], [x[1], x[5], x[9]], [x[2], x[6], x[10]]] for x in poses])
        gtRotation = np.array([RotationMatrixToEuler(x) for x in R])

    return np.concatenate([gtPosition, gtRotation], axis=-1)

def ReadBin(filename):
    ''' Read velodyne_points(format bin) '''
    cloud = np.fromfile(filename, dtype=np.float32).reshape((-1, 4))
    return cloud

def xyzi_array_to_pointcloud2(points, stamp=None, frame_id=None):
    msg = PointCloud2()

    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id

    msg.height = 1
    msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg

def main():
    pcdRoot = os.path.join(kittiRoot, "velodyne_points/data/")
    pcdPaths = [os.path.join(pcdRoot, x) for x in natsort.natsorted(os.listdir(pcdRoot))]

    poseTXT = os.path.join(kittiRoot, "pose.txt")
    gt = ReadGTPos(poseTXT)

    dataLens = len(pcdPaths)

    mapArr = np.ndarray([0, 4], dtype=np.float32)

    for i in range(dataLens):
        npArr = ReadBin(pcdPaths[i])
        npArr = PCTransform(npArr, [gt[i, 0], gt[i, 1], gt[i, 2]], [gt[i, 3], gt[i, 4], gt[i, 5]])
        mapArr = np.concatenate([mapArr, npArr])
        print("{0}/{1}".format(i+1, dataLens))

    pcmsg = xyzi_array_to_pointcloud2(mapArr, None, 'world')
    pc = pypcd.PointCloud.from_msg(pcmsg)
    pypcd.save_point_cloud_bin(pc, "umap.pcd")


if __name__ == "__main__":
    main()

