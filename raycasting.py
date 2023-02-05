import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import random


class Raycasting:
    def __init__(self):
        self.map_mesh = None
        self.mapTriangleMesh = None
        self.map_path = ""
        self.raycastingScene = o3d.t.geometry.RaycastingScene()
        self.scene = None

    def load_map(self, path):
        """Path to the map file."""
        self.map_mesh = o3d.io.read_triangle_mesh(path)
        self.map_path = path
        self.scene.scene.remove_geometry("map")

        mat = rendering.MaterialRecord()
        mat.base_color = [
            random.random(),
            random.random(),
            random.random(), 1.0
        ]
        mat.shader = "defaultLit"
        self.scene.scene.add_geometry("map", self.map_mesh, mat)
        self.mapTriangles = o3d.t.geometry.TriangleMesh.from_legacy(
            self.map_mesh)
        self.raycastingScene.add_triangles(self.mapTriangles)

    def raycast(self, origin, direction, max_distance):
        result = self.raycastingScene.raycast(origin, direction, max_distance)
        return result

    def raycast_pinhole(self, fov_deg, center, eye, up, width_px, height_px):
        """
        Raycast using a pinhole camera model.
            fov_deg: Field of view in degrees.
            center: The point the camera is looking at.
            eye: The position of the camera.
            up: Up vector of the camera.
            width_px: Width of the image in pixels.
            height_px: Height of the image in pixels.
        return: A numpy array of shape (height_px, width_px) containing the
            distance to the closest hit point. If no hit point is found, the
            value is np.inf.
        """
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            fov_deg,
            center,
            eye,
            up,
            width_px,
            height_px,
        )
        ans = self.raycastingScene.cast_rays(rays)
        return ans['t_hit']
