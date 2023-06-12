import numpy as np
import open3d as o3d

filename = "./run1retest_simplified_3d_mesh.obj"
filename2 = "cleanData_Run2.txt"
# f = open(filename, "r")
f2 = open(filename2, "r")
outdata = []

""" Reading the file and creating a numpy matrix for the function to take the x y z points as input for any given 3D structure """
for line in f2:
    data = line.split(" ")
    check = []
    for d in data:
        if d != "" and d!= " ":
            check.append(d)
    linedata = np.asarray([float(check[2])*111139, float(check[3])*111139, float(check[5])])
    outdata.append(linedata)
    # if (line[0] == 'v' and line[1] == " "):
    #     data = line.split(" ")
    #     linedata = np.asarray([float(data[1]), float(data[2]), float(data[3])])
    #     outdata.append(linedata)
""" 
    Commented out code is for processing the data from the obj file. Uncommented code is for processing the txt file.
    If processing the obj file, change the iterator to f from f2 and comment the for d in data loop and uncomment the if statement
"""
outdata = np.asarray(outdata)


""" Initializing the Point Cloud """
pcd = o3d.geometry.PointCloud()
""" Adding the Vertice Data """
pcd.points = o3d.utility.Vector3dVector(outdata)
# pcd.colors = o3d.utility.Vector3dVector(point_cloud[:,3:6]/255)
# pcd.normals = o3d.utility.Vector3dVector(point_cloud[:,6:9])
""" Commented out code adds colors for each vertice data or normal data (not available or avoided in this case) """

""" Visualizing the 3D object """
o3d.visualization.draw_geometries([pcd])

distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

""" Important: Estimating and Updating the Normals for each point in the point cloud; Necessary for creating the TriangleMesh """
pcd.estimate_normals()

""" Using Poisson Algorithm to create the Mesh; try the Ball Pivoting Algorithm if possible """
poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]

bbox = pcd.get_axis_aligned_bounding_box()
p_mesh_crop = poisson_mesh.crop(bbox)

"""  Saving the Mesh """
o3d.io.write_triangle_mesh("finally.ply", p_mesh_crop)