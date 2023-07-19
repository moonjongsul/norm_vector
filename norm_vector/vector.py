import numpy as np
import open3d as o3d

def visualize_rgbd(rgbd_image):
    # print(rgbd_image)

    # o3d.visualization.draw_geometries([rgbd_image])
    print(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    print()
    # print(o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down.
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    o3d.visualization.draw_geometries([pcd])

def get_rgbd(color: np.ndarray, depth: np.ndarray) -> o3d.geometry.RGBDImage:
    try:
        o3d_color = o3d.geometry.Image(color)
    except:
        o3d_color = o3d.geometry.Image(np.array(color))

    try:
        o3d_depth = o3d.geometry.Image(depth)
    except:
        o3d_depth = o3d.geometry.Image(np.array(depth))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth,
                                                              convert_rgb_to_intensity=False)
    return rgbd

def get_pcd(rgbd: o3d.geometry.RGBDImage) -> o3d.geometry.PointCloud:
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd,
        o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    )
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    return pcd

def rgbd2pcd(color: np.ndarray, depth: np.ndarray) -> o3d.geometry.PointCloud:
    rgbd = get_rgbd(color, depth)
    pcd = get_pcd(rgbd)
    return pcd

def get_down_pcd(pcd: o3d.geometry.PointCloud, size: float = 0.01) -> o3d.geometry.PointCloud:
    down_pcd = pcd.voxel_down_sample(voxel_size=size)
    return down_pcd

def get_norm(pcd: o3d.geometry.PointCloud, voxel_size: float = 0.01, x: int = None, y: int = None):
    pcd = get_down_pcd(pcd, voxel_size)

    pcd.normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    pcd.estimate_normals()
    return pcd
