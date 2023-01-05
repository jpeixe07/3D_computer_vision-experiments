import open3d as o3d

import cv2

camera = o3d.t.io.RealSenseSensor()

# get info from the camera itself
metadata = camera.get_metadata()
intrinsic = o3d.core.Tensor(metadata.intrinsics.intrinsic_matrix, dtype=o3d.core.Dtype.Float32)
depth_scale = metadata.depth_scale

# use this extrinsic matrix to rotate the image since frames captured with RealSense camera are upside down 
extrinsic = o3d.core.Tensor([[1,0,0,0], [0,-1,0,0], [0,0,-1,0], [0,0,0,1]], dtype=o3d.core.Dtype.Float32)

camera.start_capture()

while True:
    rgbd = camera.capture_frame(wait=True, align_depth_to_color=True)
    pcd_t = o3d.t.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic, extrinsic, depth_scale)
    
    # convert to the old version
    pcd_old = pcd_t.to_legacy()
    o3d.visualization.draw_geometries([pcd_t], zoom=0.5)
    # do stuff
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
camera.stop()
