import open3d as o3d
import open3d.visualization.rendering as rendering
import matplotlib.pyplot as plt
import numpy as np

print('Set material shader')
mtl = o3d.visualization.rendering.MaterialRecord()
mtl.base_color = [1.0, 1.0, 1.0, 0.3]
mtl.shader = "defaultUnlit"

mesh_size = 1.0
small_sphere      = o3d.geometry.TriangleMesh.create_sphere(mesh_size/20.0)  # trajectory points
mesh_sphere       = o3d.geometry.TriangleMesh.create_sphere(mesh_size/5.0)  # successful predict points
mesh_sphere_fear  = o3d.geometry.TriangleMesh.create_sphere(mesh_size/5.0)  # unsuccessful predict points
mesh_box          = o3d.geometry.TriangleMesh.create_box(mesh_size, mesh_size, mesh_size)  # end 

small_sphere.paint_uniform_color([0.99, 0.2, 0.1])  # green
mesh_sphere.paint_uniform_color([0.4, 1.0, 0.1])
mesh_sphere_fear.paint_uniform_color([1.0, 0.64, 0.0])
mesh_box.paint_uniform_color([1.0, 0.64, 0.1])

camera_width = 640
camera_height = 480
render = rendering.OffscreenRenderer(camera_width, camera_height)
render.scene.set_background([0.0, 0.0, 0.0, 1.0])  # RGBA
render.scene.scene.enable_sun_light(False)

# print('compute vertex normals')
# small_sphere.compute_vertex_normals()
# mesh_sphere.compute_vertex_normals()
# mesh_sphere_fear.compute_vertex_normals()
# mesh_box.compute_vertex_normals()

