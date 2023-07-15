# Create ROSBAG with the IMU and uncompressed video data
#
# Description:
#   Create data structure containing IMU and ARKit data.
#
# Arguments:
#   - Data folder.
#   - sequences to bag
#
# Copyright (C) 2018 Santiago Cortes
#
# This software is distributed under the GNU General Public
# License (version 2 or later); please refer to the file
# License.txt, included with the software, for details.

# Import system libraries
import sys, os
import csv
import numpy as np

# Import ros libraries.
from cv_bridge import CvBridge, CvBridgeError
import rosbag2_py
import rclpy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
import math
import shutil

# Import opencv
import cv2

# Read arguments
# Folder
folder = sys.argv[1]
# If number argument is added, just build that rosbag. Else, build all.
try:
    sys.argv[2]
except:
    tst = range(1, 24)
else:
    tst = [int(sys.argv[2])]

print("Creating rosbags for sequences in " + str(tst))

# Delete first frames
start = 0.5

# Loop trough all sequences.
for i in iter(tst):
    # Find folder
    dir = folder + "/advio-" + str(i).zfill(2) + "/iphone"
    print(dir)

    bag_path = folder + "/../bags" + "/advio-" + str(i).zfill(2)

    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)

    # Initialize bag, data and video reader.
    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=bag_path, storage_id="sqlite3"
    )
    converter_options = rosbag2_py._storage.ConverterOptions("", "")
    writer.open(storage_options, converter_options)

    image_topic_info = rosbag2_py._storage.TopicMetadata(
        name="/cam0/image_raw", type="sensor_msgs/msg/Image", serialization_format="cdr"
    )
    writer.create_topic(image_topic_info)

    imu_topic_info = rosbag2_py._storage.TopicMetadata(
        name="/imu0", type="sensor_msgs/msg/Imu", serialization_format="cdr"
    )
    writer.create_topic(imu_topic_info)

    index = 0
    csvfile = open(dir + "/" + "imu-gyro.csv")
    datareader = csv.reader(csvfile, delimiter=" ", quotechar="|")

    

    # Initialize bridge.
    bridge = CvBridge()

    # Read video and check for number of frames.
    vidcap = cv2.VideoCapture(dir + "/" + "frames.mov")
    vlength = int(vidcap.get(cv2.CAP_PROP_FRAME_COUNT))

    # For each row in the data matrix
    for row in datareader:
        # Read data row
        sp = row[0].split(",")
        stamp = float(sp[0])

        # If the row correspond to a frame
        if int(float(sp[1])) == 7:
            # Read next frame and check.
            success, image = vidcap.read()
            if success and float(sp[0]) > start:
                # Stamp = rospy.rostime.Time.from_sec(stamp)
                # Put image in correct orientation and convert into grayscale.
                gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # gray_image = cv2.transpose(gray_image)
                # gray_image = cv2.flip(gray_image, +1)

                # Print progress.
                if index % 500 == 0:
                    print(str(int(np.round(100 * float(index) / float(vlength)))) + "%")
                index = index + 1

                # Create ros image and put frame in it.
                Img = Image()
                Img = bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
                Img.header.stamp = Time(
                    sec=int(math.floor(stamp)),
                    nanosec=int((stamp - math.floor(stamp)) * pow(10, 9)),
                )

                # Put image in rosbag.
                # bag.write("/cam0/image_raw", Img, Stamp)

                writer.write(
                    "/cam0/image_raw",
                    serialize_message(Img),
                    int(stamp * pow(10, 9)),
                )

        # If the row corresponds to an IMU measurement.
        elif int(float(sp[1])) == 34 and float(sp[0]) > start:

            # Read acceleration and rotational rate.
            gyr = sp[2:5:1]
            acc = sp[5:8:1]

            # Create imu ros structure and put in the bias corrected values.
            imu = Imu()
            imu.header.stamp = Time(
                sec=int(math.floor(stamp)),
                nanosec=int((stamp - math.floor(stamp)) * pow(10, 9)),
            )
            imu.angular_velocity.x = float(gyr[0]) + 0.0067
            imu.angular_velocity.y = float(gyr[1]) - 0.0070
            imu.angular_velocity.z = float(gyr[2]) + 0.0065

            imu.linear_acceleration.x = float(acc[0]) - 0.0407
            imu.linear_acceleration.y = float(acc[1]) + 0.0623
            imu.linear_acceleration.z = float(acc[2]) - 0.1017
            writer.write(
                "/imu0",
                serialize_message(imu),
                int(stamp * pow(10, 9)),
            )

print("Process complete")
