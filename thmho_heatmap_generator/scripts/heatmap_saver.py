#!/usr/bin/env python3
from typing import Protocol
import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np


class HeatmapSaver(object):
    def __init__(self):
        self.msg = OccupancyGrid()
        self.heatmap_subscriber = rospy.Subscriber(
            "/heatmap", OccupancyGrid, self.save_heatmap)

    def save_heatmap(self, msg):
        map_id=msg.header.seq
        width = msg.info.width
        height = msg.info.height
        heatmap = np.reshape(msg.data, (width, height))
        plt.imshow(heatmap, cmap='coolwarm', interpolation='nearest')
        plt.savefig("./heatmap/"+str(map_id)+".png")

if __name__ == '__main__':
    rospy.init_node('HeatmapSaver', anonymous=True)
    heatmap = HeatmapSaver()
    rospy.spin()
