import rospy
import tf2_ros
import numpy as np
from tric_navigation.msg import PLC_Feedback
import rospkg
import os
import traceback
import json
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from tric_navigation.msg import AutoNavCommand
from tric_navigation.msg import ManualNavCommand
from sensor_msgs.msg import NavSatFix
import pandas as pd
from geometry_msgs.msg import Pose
from datetime import datetime
from rospy_message_converter import json_message_converter
import time
import re
import subprocess
from geopy.distance import geodesic
import warnings

class Data_Logger:
    def __init__(self):
        try: 
            rospy.init_node("data_logger", anonymous=True)
            self.rospack = rospkg.RosPack()
            self.ros_package_path = self.rospack.get_path('tric_navigation')
            self.tf_buffer = tf2_ros.Buffer()
            self.transform_listener = tf2_ros.TransformListener(self.tf_buffer)
            self.log_folder_path = self.ros_package_path + "/src/logs/"
            self.log_file_path = None
            self.log_interval_time = 3
            self.save_log_interval_time = 30
            self.gps_velocity_list = []
            self.prev_gps_head_data_msg = None
            self.prev_gps_head_data_msg_time = None
            self.log_reset_hour = 12
            self.curr_log_file_path = None
            self.log_column_list = ["Time",
                                "Date Time",
                                "Joystick Control",
                                "Auto Wing%/Boom Control",
                                "Control Stop",
                                "GPS Head Position",
                                "GPS Head Status",
                                "GPS Tail Status",
                                "Linear Velocity GPS",
                                "Current Waypoint Index",
                                "Navigation Timeout",
                                "Treatment Area",
                                "Left Wing Command",
                                "Right Wing Command",
                                "Boom Wing Command",
                                "All UVC Array Status",
                                "UVC Array 1 Status",
                                "UVC Array 2 Status",
                                "UVC Array 3 Status",
                                "UVC Array 4 Status",
                                "UVC Array 5 Status",
                                "UVC Array 6 Status",
                                "UVC Array 7 Status",
                                "PLC Feedback Msg",
                                "GPS Head Transform",
                                "GPS Tail Transform",
                                "rospy-debug",
                                "rospy-info",
                                "rospy-warn",
                                "rospy-error",
                                "rospy-fatal",
                                "USB Devices",
                                "Manual Nav Msg",
                                "Auto Nav Msg",
                                "Waypoint"]
            warnings.simplefilter(action='ignore', category=FutureWarning)

            self.setup_log()
            self.subscribe_to_topics()
        except Exception as e:
            error = traceback.format_exc()
            rospy.logerr("ERROR: %s",  error)

    def setup_log(self):
        
        self.unwritten_single_row_df = pd.DataFrame(columns=self.log_column_list)

        log_file_list = os.listdir(self.log_folder_path)
        curr_date_time = datetime.now()
        
        
        for log_file in log_file_list:
            if(log_file.endswith(".csv")):
                log_file = log_file.strip(".csv")
                log_file_date_time_list = log_file.split('_')
                log_file_date_time = datetime(int(log_file_date_time_list[0]),int(log_file_date_time_list[1]), int(log_file_date_time_list[2]), 12)
                if(((curr_date_time - log_file_date_time).seconds/360) <= 24):
                    self.log_file_date_time = log_file_date_time
                    self.curr_log_file_path = self.log_folder_path + log_file + ".csv"
                    self.log_df = pd.read_csv(self.curr_log_file_path)
                    break        
        
        if(self.curr_log_file_path == None):
            self.curr_log_file_path = self.log_folder_path + curr_date_time.strftime("%Y_%m_")
            
            if(curr_date_time.hour < 12):
                self.curr_log_file_path += str(curr_date_time.day-1)+".csv"
                self.log_file_date_time = datetime(curr_date_time.year ,curr_date_time.month, curr_date_time.day-1, 12)
            else:
                self.curr_log_file_path += str(curr_date_time.day)+".csv"
                self.log_file_date_time = datetime(curr_date_time.year ,curr_date_time.month, curr_date_time.day, 12)
            self.log_df = pd.DataFrame(columns=self.log_column_list)
                

    def subscribe_to_topics(self):
        rospy.Subscriber("/tric_navigation/right_wing_command", Float64, self.right_wing_command_callback)
        rospy.Subscriber("/tric_navigation/left_wing_command", Float64, self.left_wing_command_callback)
        rospy.Subscriber("/tric_navigation/boom_command", Float64, self.boom_command_callback)
        rospy.Subscriber("/tric_navigation/manual_nav_command", ManualNavCommand, self.manual_nav_command_callback)
        rospy.Subscriber("/tric_navigation/uvc_light_status", String, self.uvc_light_status_callback)
        rospy.Subscriber("/tric_navigation/auto_nav_command", AutoNavCommand, self.auto_nav_command_callback)
        rospy.Subscriber("/tric_navigation/joystick_control", Bool, self.joystick_control_callback)
        #rospy.Subscriber("/tric_navigation/treatment_area", Bool, self.treatment_area_callback)
        rospy.Subscriber("/tric_navigation/navigation_timeout", Bool, self.navigation_timeout_callback)
        rospy.Subscriber("/tric_navigation/waypoint", Pose, self.waypoint_update_callback)
        rospy.Subscriber("/tric_navigation/plc_feedback", PLC_Feedback, self.plc_feedback_callback)
        rospy.Subscriber("/tric_navigation/gps/head_location", NavSatFix, self.gps_head_data_callback)
        rospy.Subscriber("/tric_navigation/gps/head_status", String, self.gps_head_status_callback)
        rospy.Subscriber("/tric_navigation/gps/tail_status", String, self.gps_tail_status_callback)
        rospy.Subscriber("/rosout", Log, self.rosout_callback)

        rospy.Timer(rospy.Duration(self.log_interval_time), self.log_data)
        rospy.Timer(rospy.Duration(self.save_log_interval_time), self.save_log)
        rospy.spin()

    def right_wing_command_callback(self, right_wing_command_msg):
        self.unwritten_single_row_df["Right Wing Command"] = right_wing_command_msg.data

    def left_wing_command_callback(self, left_wing_command_msg):
        self.unwritten_single_row_df["Left Wing Command"] = left_wing_command_msg.data

    def boom_command_callback(self, boom_command_msg):
        self.unwritten_single_row_df["Boom Command"] = boom_command_msg.data

    def manual_nav_command_callback(self, manual_nav_command_msg):
        manual_nav_command_json = json_message_converter.convert_ros_message_to_json(manual_nav_command_msg)
        self.unwritten_single_row_df["Manual Nav Msg"] = manual_nav_command_json

        self.unwritten_single_row_df["Auto Wing%/Boom Control"] = bool(manual_nav_command_msg.wing_boom_auto_control_enable)
        self.unwritten_single_row_df["Control Stop"] = bool(manual_nav_command_msg.control_stop)

    def uvc_light_status_callback(self, uvc_light_status_msg):
        for idx, uvc_switch in enumerate(uvc_light_status_msg.data):
            column_str =  "UVC Array %d Status" % (idx+1)
            if(uvc_switch == "0"):
                uvc_status_bool= False
            elif(uvc_switch == "1"):
                uvc_status_bool = True
            self.unwritten_single_row_df[column_str] = uvc_status_bool
        if "0" in uvc_light_status_msg.data:
            self.unwritten_single_row_df["All UVC Array Status"] = False
        else:
            self.unwritten_single_row_df["All UVC Array Status"] = True


    def auto_nav_command_callback(self, auto_nav_command_msg):
        auto_nav_command_json = json_message_converter.convert_ros_message_to_json(auto_nav_command_msg)
        self.unwritten_single_row_df["Auto Nav Msg"] = auto_nav_command_json

    def joystick_control_callback(self, joystick_control_msg):
        self.unwritten_single_row_df["Joystick Control"] = bool(joystick_control_msg.data)

    def gps_head_data_callback(self, gps_head_data_msg):
        gps_head_json = json_message_converter.convert_ros_message_to_json(gps_head_data_msg)
        self.unwritten_single_row_df["GPS Head Position"] = gps_head_json
        curr_gps_sample_time = time.time()
        if(self.prev_gps_head_data_msg != None):
            distance = geodesic((gps_head_data_msg.latitude,gps_head_data_msg.longitude), (self.prev_gps_head_data_msg.latitude, self.prev_gps_head_data_msg.longitude)).meters
            velocity = distance / (curr_gps_sample_time - self.prev_gps_head_data_msg_time)
            self.gps_velocity_list.append(velocity)
        self.prev_gps_head_data_msg = gps_head_data_msg
        self.prev_gps_head_data_msg_time = curr_gps_sample_time

    def gps_head_status_callback(self, gps_head_status_msg):
        self.unwritten_single_row_df["GPS Head Status"] = gps_head_status_msg.data

    def gps_tail_status_callback(self, gps_tail_status_msg):
        self.unwritten_single_row_df["GPS Tail Status"] = gps_tail_status_msg.data

    def navigation_timeout_callback(self, navigation_timeout_msg):
        self.unwritten_single_row_df["Navigation Timeout"] = bool(navigation_timeout_msg.data)

    def waypoint_update_callback(self, waypoint_update_msg):
        waypoint_update_json = json_message_converter.convert_ros_message_to_json(waypoint_update_msg)
        self.unwritten_single_row_df["Waypoint"] = waypoint_update_json
        self.unwritten_single_row_df["Treatment Area"] = waypoint_update_msg.treatment_area

    def plc_feedback_callback(self, plc_feedback_msg):
        plc_feedback_json = json_message_converter.convert_ros_message_to_json(plc_feedback_msg)
        self.unwritten_single_row_df["PLC Feedback Msg"] = plc_feedback_json

    def rosout_callback(self, rosout_msg):
        if(int(rosout_msg.level) == 1):
            self.unwritten_single_row_df["rospy-debug"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif(int(rosout_msg.level) == 2):
            self.unwritten_single_row_df["rospy-info"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif(int(rosout_msg.level) == 4):
            self.unwritten_single_row_df["rospy-warn"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif(int(rosout_msg.level) == 8):
            self.unwritten_single_row_df["rospy-error"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
        elif(int(rosout_msg.level) == 16):
            self.unwritten_single_row_df["rospy-fatal"] = json_message_converter.convert_ros_message_to_json(rosout_msg)
    
        



    def log_data(self, timer):
        
        self.unwritten_single_row_df["Time"] = time.time()
        self.unwritten_single_row_df["Date Time"]= datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")
        if(self.gps_velocity_list):
            self.unwritten_single_row_df["Linear Velocity GPS"]= np.mean(self.gps_velocity_list)
        self.gps_velocity_list = []
        robot_position_head, robot_position_tail = self.get_robot_position(get_tail=True)
        if(robot_position_head != None):
            self.unwritten_single_row_df["GPS Head Transform"] = json_message_converter.convert_ros_message_to_json(robot_position_head)
        if(robot_position_tail != None):
            self.unwritten_single_row_df["GPS Tail Transform"] = json_message_converter.convert_ros_message_to_json(robot_position_tail)
        self.unwritten_single_row_df["USB Devices"] = self.get_usb_devices()
        
            
        #self.log_df.concat(self.unwritten_single_row_df)
        self.log_df = pd.concat([self.log_df, self.unwritten_single_row_df])
        
        self.unwritten_single_row_df = pd.DataFrame(columns=self.log_column_list, index = [0])

    def get_usb_devices(self):
        
        usb_device_re = re.compile(b"Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
        df = subprocess.check_output("lsusb")
        usb_devices = [] 
        for i in df.split(b'\n'):
            if i:
                info = usb_device_re.match(i)
                if info:
                    dinfo = info.groupdict()
                    dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
                    usb_devices.append(str(dinfo))
        
        return "".join(usb_devices)

    def get_robot_position(self, get_tail=False):
        #lookup latest transform to get robot position
        try:
            if(get_tail == False):
                head_transform = self.tf_buffer.lookup_transform(
                    "datum", "front_wheel", rospy.Time(0)).transform
                return head_transform
            else:
                head_transform = self.tf_buffer.lookup_transform("datum", "front_wheel", rospy.Time(0)).transform
                tail_transform = self.tf_buffer.lookup_transform("datum", "rear_wheel", rospy.Time(0)).transform

                return (head_transform, tail_transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
            return (None, None)

    def save_log(self, timer):
        
        curr_date_time = datetime.now()
        if(((curr_date_time - self.log_file_date_time).seconds/360) <= 24):
            self.setup_log()
            return
        else:
            self.log_df.to_csv(self.curr_log_file_path, index = False)



if __name__ == "__main__":
    data_logger = Data_Logger()
