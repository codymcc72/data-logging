import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
from rosgraph_msgs.msg import Log
from datetime import datetime
import pandas as pd
from rospy_message_converter import json_message_converter
import time
import os  

class SimpleDataLogger:
    def __init__(self):
        try:
            rospy.init_node("simple_data_logger", anonymous=True)
            self.log_column_list = ["Time", "Date Time", "Joystick Control", "GPS Head Position", "Treatment Area",
                                    "rospy-debug", "rospy-info", "rospy-warn", "rospy-error", "rospy-fatal"]
            self.log_df = pd.DataFrame(columns=self.log_column_list)
            self.unwritten_single_row_df = pd.DataFrame(columns=self.log_column_list, index=[0])

            self.setup_log()
            self.subscribe_to_topics()
        except Exception as e:
            rospy.logerr("ERROR: %s", str(e))

    def setup_log(self):
        self.log_file_path = "your_log_file_path.csv"

    def subscribe_to_topics(self):
        rospy.Subscriber("/tric_navigation/joystick_control", Bool, self.joystick_control_callback)
        rospy.Subscriber("/tric_navigation/gps/head_location", NavSatFix, self.gps_head_data_callback)
        rospy.Subscriber("/tric_navigation/treatment_area", Bool, self.treatment_area_callback)
        rospy.Subscriber("/rosout", Log, self.rosout_callback)

        rospy.Timer(rospy.Duration(3), self.log_data)
        rospy.spin()

    def joystick_control_callback(self, joystick_control_msg):
        self.unwritten_single_row_df["Joystick Control"] = bool(joystick_control_msg.data)

    def gps_head_data_callback(self, gps_head_data_msg):
        gps_head_json = json_message_converter.convert_ros_message_to_json(gps_head_data_msg)
        self.unwritten_single_row_df["GPS Head Position"] = gps_head_json

    def treatment_area_callback(self, treatment_area_msg):
        self.unwritten_single_row_df["Treatment Area"] = bool(treatment_area_msg.data)

    def rosout_callback(self, rosout_msg):
        if int(rosout_msg.level) == 1:
            self.unwritten_single_row_df["rospy-debug"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif int(rosout_msg.level) == 2:
            self.unwritten_single_row_df["rospy-info"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif int(rosout_msg.level) == 4:
            self.unwritten_single_row_df["rospy-warn"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif int(rosout_msg.level) == 8:
            self.unwritten_single_row_df["rospy-error"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif int(rosout_msg.level) == 16:
            self.unwritten_single_row_df["rospy-fatal"] = json_message_converter.convert_ros_message_to_json(rosout_msg)

    def log_data(self, timer):
        self.unwritten_single_row_df["Time"] = time.time()
        self.unwritten_single_row_df["Date Time"] = datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")

        # Check if the CSV file exists
        csv_exists = os.path.exists(self.log_file_path)

        # Save log data to CSV file
        if not csv_exists:
            # If the file doesn't exist, create a new one
            self.log_df.to_csv(self.log_file_path, index=False)
        else:
            # If the file exists, append data to it
            self.log_df.to_csv(self.log_file_path, mode='a', header=False, index=False)

        # Reset the DataFrame for the next cycle
        self.unwritten_single_row_df = pd.DataFrame(columns=self.log_column_list, index=[0])

if __name__ == "__main__":
    data_logger = SimpleDataLogger()
