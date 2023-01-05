import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])


#camera = o3d.t.io.RealSenseSensor() #So pra ler os parametros intrinsicos da camera, por enquanto a gente ainda nao sabe
                                    #Transformar direto, tem que salvar em um arquivo primeiro

#metadata = camera.get_metadata() #aparentemente precisa do video
#print(metadata)
#intr = o3d.open3d.camera.PinholeCameraIntrinsic(640, 480, fx=463.889, fy=463.889, cx=320, cy=240)  
#Salva os parametros intrinsicos 

color_raw = o3d.io.read_image('C:/Users/jpedr/Documents/Python/RGBD_realsense-experiments/depth_color/color440.jpg')
depth_raw = o3d.io.read_image('C:/Users/jpedr/Documents/Python/RGBD_realsense-experiments/depth_color/depth440.jpg')
#Le e transforma em uma imagem do tipo RGB+DEPTH para receber as funcoes do open3D
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
print(rgbd_image)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, 
                                       o3d.camera.PinholeCameraIntrinsic(
                                                    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
#pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intr)

#pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, o3d.camera.PinholeCameraIntrinsic(intrinsic))

pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

#o3d.visualization.draw([pcd])
#o3d.visualization.draw_geometries([pcd])

#0.0000001 fininho?? para esse parametro, valores altos vao fazer o pointcloud sumir
#0.0000009 é o padrao para o voxel_size
downpcd = pcd.voxel_down_sample(voxel_size=0.0000054232)
#o3d.visualization.draw_geometries([downpcd])
#print(np.asarray(downpcd.points))


#OUTLIER REMOVAL
#cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=200,
#                                                    std_ratio=0.01)
#display_inlier_outlier(downpcd, ind)

#ESTIMANDO PLANO NORMAL
#downpcd.estimate_normals(
#    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
#o3d.visualization.draw_geometries([downpcd], point_show_normal=True)



#CRIANDO MESH ATRAVES DO ALPHA_SHAPE
#alpha = 0.03
#print(f"alpha={alpha:.3f}")
#print('Running alpha shapes surface reconstruction ...')
#mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
#    pcd, alpha)
#mesh.compute_triangle_normals(normalized=True)
#print("Displaying reconstructed mesh ...")
#o3d.visualization.draw_geometries([mesh], mesh_show_back_face=True)




##CLUSTERING BROOO
###Brinca com os parametros, que nao ta pronto ainda
#with o3d.utility.VerbosityContextManager(
#       o3d.utility.VerbosityLevel.Debug) as cm:
#    labels = np.array(
#        downpcd.cluster_dbscan(eps=0.0000055, min_points=20, print_progress=True))

#max_label = labels.max()
#print(f"point cloud has {max_label + 1} clusters")
#colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
#colors[labels < 0] = 0
#downpcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
#o3d.visualization.draw_geometries([downpcd])#alterar para downpcd viu







#SEGMENTACAO DE PLANOOOOOOO
plane_model, inliers = downpcd.segment_plane(distance_threshold=0.000005,
                                        ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

inlier_cloud = downpcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = downpcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

