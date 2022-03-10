
#!/usr/bin/env python3
from pyngsi.sink import SinkOrion
from pyngsi.ngsi import DataModel
from datetime import datetime
import os
import platform


def ngsi_model(msg):
    """Function to standardize the OccupancyGrid message to NGSIv2 model

    Args:
         msg (nav_msgs.msg.OccupancyGrid): Occupancy grid
    """
    sink = SinkOrion()
    _dateTime = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    _id = str(msg.header.seq)

    ngsi_body = DataModel(id=_id, type="urn:ngsi:FraunhoferIPA:THMHO")
    ngsi_body.add("timestamp", {
        "dateTime": _dateTime,
        "format": "dd/MM/yyyy",
        "timezoneId": "Europe/Berlin"
    })

    ngsi_body.add("ioTDeviceInformation", {
        "sensorId": os.uname().nodename,
        "sensorType": platform.processor()
    })
    ngsi_body.add("heatmap_data", {
        "header": {
            "seq": int(msg.header.seq),
            "stamp": {
                  "secs": int(msg.header.stamp.secs),
                  "nsecs": int(msg.header.stamp.nsecs)
                  },
            "frame_id": str(msg.header.frame_id)
        },
        "info": {
            "map_load_time": {
                "secs": int(msg.info.map_load_time.secs),
                "nsecs": int(msg.info.map_load_time.nsecs)
            },
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin": {
                "position": {
                    "x": float(msg.info.origin.position.x),
                    "y": float(msg.info.origin.position.y),
                    "z": float(msg.info.origin.position.z)
                },
                "orientation": {
                    "x": float(msg.info.origin.orientation.x),
                    "y": float(msg.info.origin.orientation.y),
                    "z": float(msg.info.origin.orientation.z),
                    "w": float(msg.info.origin.orientation.w)
                }
            }
        },
        "data": msg.data
    })
    sink.write(ngsi_body.json())
