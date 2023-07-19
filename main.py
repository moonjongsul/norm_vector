import cv2
import numpy as np
import open3d as o3d

from ketisdk.sensor.realsense_sensor import RSSensor
import norm_vector as nv

def main():
    sensor = RSSensor()
    sensor.start()

    k = [[sensor.info.fx, 0, sensor.info.cx],
         [0, sensor.info.fy, sensor.info.cy],
         [0,              0,              1]]
    intrinsic = np.array(k)
    # print(intrinsic)

    while True:
        try:
            rgb, depth = sensor.get_data()
        except:
            continue

        pcd = nv.rgbd2pcd(rgb, depth)
        pcd = nv.get_norm(pcd)

        print(np.asarray(pcd.points).shape)
        print(np.asarray(pcd.colors).shape)
        o3d.visualization.draw_geometries([pcd], point_show_normal=True)
        print()
        # cv2.imshow('cv', rgb[:, :, ::-1])
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     cv2.destroyAllWindows()
        #     break


if __name__ == '__main__':
    main()
