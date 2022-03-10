#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from ngsi_model import ngsi_model
from datetime import datetime
from pyngsi.ngsi import DataModel


class HeatmapNGSI():
    """ROS message is converted to NGSIv2 messages for orion context broker
    """

    def __init__(self):
        self.msg = OccupancyGrid()
        self.heatmap_subscriber = rospy.Subscriber(
            "/heatmap_generator/heatmap", OccupancyGrid, self.build_ngsi_msgs)

    def build_ngsi_msgs(self, msg):
        """Function to build standardized NGSIv2 from OccupancyGrid

        Args:
            msg (nav_msgs.msg.OccupancyGrid): Occupancy grid
        """
        ngsi_model(msg)


if __name__ == '__main__':
    rospy.init_node('ngsiv2_translator_node', anonymous=True)
    ngsi_msg = HeatmapNGSI()
    rospy.spin()
