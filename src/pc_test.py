#!/usr/bin/env python3

import numpy as np
from sensor_msgs.msg import PointCloud2, ChannelFloat32, PointField
from geometry_msgs.msg import Point32
import rospy



class PointCloudComputer:

    def __init__(self, nrows: int, ncols: int, vertical_fov: float, horizontal_fov: float):
        self.nrows = nrows
        self.ncols = ncols
        self.fy = nrows/(2*np.tan((0.5*np.pi)*(vertical_fov/nrows)))
        self.fx = ncols/(2*np.tan((0.5*np.pi)*(horizontal_fov/nrows)))

        self.row_arr = np.tile(np.arange(self.nrows), (self.ncols, 1)).T
        self.col_arr = np.tile(np.arange(self.ncols), (self.nrows, 1))
        self.zero = np.zeros((nrows, ncols))
        self.msg = PointCloud2()
        self.msg.height=1
        self.msg.width=ncols*nrows
        self.msg.fields = [PointField()]*3
        self.msg.fields[0].name = "X"
        self.msg.fields[1].name = "Y"
        self.msg.fields[2].name = "Z"
        for i in range(3):
            self.msg.fields[i].offset = i*(nrows*ncols)
            self.msg.fields[i].datatype=7
            self.msg.fields[i].count = nrows*ncols
        # self.msg.points = [Point32()]*(self.nrows*self.ncols)
        # self.msg.channels = [ChannelFloat32()]
        # self.msg.channels[0].values = [0]*(self.nrows*self.ncols)




    def numpy_to_pcmsg(self, depth: np.ndarray, amplitude: np.ndarray) -> PointCloud2:
        valid_idxs = np.where(amplitude > 30)
        z_arr = np.where(amplitude > 30, depth, self.zero)
        x_arr = np.where(amplitude > 30, (((self.ncols/2) - self.col_arr) / self.fx) * z_arr, self.zero)
        y_arr = np.where(amplitude > 30, (((self.nrows/2) - self.row_arr) / self.fy) * z_arr, self.zero)
        self.msg.data = list(x_arr.flatten()) + list(y_arr.flatten()) + list(z_arr.flatten())
        # count = 0
        # self.msg.channels[0].values = list(depth.flatten())
        # for row_idx in range(self.nrows):
        #     for col_idx in range(self.ncols):
        #         self.msg.points[count].x = x_arr[row_idx, col_idx]
        #         self.msg.points[count].y = y_arr[row_idx, col_idx]
        #         self.msg.points[count].z = z_arr[row_idx, col_idx]
        #         count += 1
                
        self.msg.header.stamp = rospy.Time.now()
        self.msg.header.frame_id = "pointcloud"
        return self.msg


if __name__ == "__main__":

    rospy.init_node("random_pc_pub")
    pub = rospy.Publisher("/pointcloud2", PointCloud2, queue_size=10)

    calc = PointCloudComputer(180, 240, 50.4, 64.3)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        fake_depth = np.ones((180,240))
        fake_amp = np.random.rand(180,240)+30
        pub.publish(calc.numpy_to_pcmsg(fake_depth, fake_amp))