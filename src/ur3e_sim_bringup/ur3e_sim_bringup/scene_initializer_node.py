import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class SceneInitializerNode(Node):
    def __init__(self):
        super().__init__('scene_initializer_node')
        self._done = False
        self._step = 0
        self._client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self._retry_counter = 0
        self._future = None
        self._timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self) -> None:
        if self._step == 0:
            if not self._client.wait_for_service(timeout_sec=0.5):
                self._retry_counter += 1
                if self._retry_counter > 30:
                    self.get_logger().error('Planning scene service unavailable after 30s')
                    self._done = True
                    self._timer.cancel()
                return
            self._timer.cancel()
            self.get_logger().info('Connected to planning scene service')
            self._send_ground_plane()
            self._step = 1
            self._timer.reset()
            return

        if self._step == 1:
            if self._future is not None and self._future.done():
                if self._future.result() and self._future.result().success:
                    self.get_logger().info('ground_plane added to planning scene')
                else:
                    self.get_logger().error('Failed to add ground_plane')
                self._future = None
                self._send_target_cube()
                self._step = 2
            return

        if self._step == 2:
            if self._future is not None and self._future.done():
                if self._future.result() and self._future.result().success:
                    self.get_logger().info('target_cube added to planning scene')
                else:
                    self.get_logger().error('Failed to add target_cube')
                self._done = True
                self._timer.cancel()

    def _send_ground_plane(self) -> None:
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = 'ground_plane'
        obj.primitives.append(SolidPrimitive())
        obj.primitives[0].type = SolidPrimitive.BOX
        obj.primitives[0].dimensions = [2.0, 2.0, 0.05]
        obj.primitive_poses.append(Pose())
        obj.primitive_poses[0].position.z = -0.025
        obj.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects.append(obj)

        self._future = self._client.call_async(req)

    def _send_target_cube(self) -> None:
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = 'target_cube'
        obj.primitives.append(SolidPrimitive())
        obj.primitives[0].type = SolidPrimitive.SPHERE
        obj.primitives[0].dimensions = [0.015]
        obj.primitive_poses.append(Pose())
        obj.primitive_poses[0].position.x = 0.35
        obj.primitive_poses[0].position.y = 0.0
        obj.primitive_poses[0].position.z = 0.05
        obj.operation = CollisionObject.ADD

        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects.append(obj)

        self._future = self._client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = SceneInitializerNode()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
