#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

import requests
from datetime import datetime


class HeatmapSaver(object):
    def __init__(self):
        self.url = 'http://localhost:5000/insert'
        self.msg = OccupancyGrid()
        self.heatmap_subscriber = rospy.Subscriber(
            "/heatmap_generator/heatmap", OccupancyGrid, self.save_heatmap)

    def save_heatmap(self, msg):
        body = {
            'date': datetime.now().isoformat(),
            'data': msg.data,
            'width': msg.info.width,
            'height': msg.info.height    
        }
        requests.post(self.url, json=body)


if __name__ == '__main__':
    rospy.init_node('heatmap_saver', anonymous=True)
    heatmap = HeatmapSaver()
    rospy.spin()
