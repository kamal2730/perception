import rclpy
from rclpy.node import Node
from pcc.msg import PointCloud2Array   # custom msg
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class PCDSubscriber(Node):
    def __init__(self):
        super().__init__('pcd_subscriber')

        self.subscription = self.create_subscription(
            PointCloud2Array,
            '/objects/cluster_array',   # topic where clusters are published
            self.listener_callback,
            10)

        self.get_logger().info("Subscribed to /objects/cluster_array")

    def listener_callback(self, msg: PointCloud2Array):
        self.get_logger().info(f"Received {len(msg.clouds)} clusters")

        for i, cloud in enumerate(msg.clouds):
            points = list(point_cloud2.read_points(cloud, field_names=("x","y","z"), skip_nans=True))
            self.get_logger().info(f"Cluster {i}: {len(points)} points")

def main(args=None):
    rclpy.init(args=args)
    node = PCDSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
