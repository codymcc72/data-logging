import rospy
import rospkg
import os
import pandas as pd
import requests
import json
from datetime import datetime
import pytz

class Airtable_Log_Uploader:
    def __init__(self):
        rospy.init_node("airtable_log_uploader", anonymous=True)
        self.rospack = rospkg.RosPack()
        self.ros_package_path = self.rospack.get_path('tric_navigation')
        self.log_folder_path = self.ros_package_path + "/src/logs/"
        self.upload_log_interval_time = 10

        self.row_turn_log_table_id = rospy.get_param("row_turn_log_table_id")
        self.treatment_log_table_id = rospy.get_param("treatment_log_table_id")
        self.airtable_api_key = rospy.get_param("airtable_api_key")
        self.log_base_id = rospy.get_param("log_base_id")

        self.airtable_upload_loop()

    def airtable_upload_loop(self):
        rospy.Timer(rospy.Duration(self.upload_log_interval_time), self.check_for_new_treatment)
        rospy.spin()

    
    def get_gps_fix_percentage(self, log_df):
        head_fix_count = (log_df["GPS Head Status"]=='Fix').sum()
        head_total_count = log_df["GPS Head Status"].sum()
        head_fix_percentage = ( head_fix_count/ head_total_count)*100
        tail_fix_count = (log_df["GPS Tail Status"]=='Fix').sum()
        tail_total_count = log_df["GPS Tail Status"].sum()
        tail_fix_percentage = ( tail_fix_count/ tail_total_count)*100
        return head_fix_percentage, tail_fix_percentage

    def check_for_new_treatment(self, timer):
        
        log_file_list = os.listdir(self.log_folder_path)
        #self.airtable_row_turn_log_df = self.convert_to_dataframe(self.airtable_download(self.row_turn_log_table_id, params_dict={}, api_key=self.airtable_api_key, base_id=self.log_base_id))
        self.airtable_treatment_log_df = self.convert_to_dataframe(self.airtable_download(table=self.treatment_log_table_id, params_dict={"view": "Treatments to Upload Robot Data"},api_key=self.airtable_api_key, base_id=self.log_base_id))
        for treatment_log_row_index, treatment_log_row in self.airtable_treatment_log_df.iterrows():
        #    if(treatment_log_row["Logged Treatment Start Time"] != None and treatment_log_row["Logged Treatment End Time"] != None):
            for log_file in log_file_list:
                if(log_file.endswith(".csv")):

                    log_file_treatment_date_list = log_file.strip(".csv").split('_')
                    treatment_log_treatment_date = treatment_log_row["Treatment Date"]
                    treatment_log_treatment_date_list = treatment_log_treatment_date.split('-')


                    if(log_file_treatment_date_list == treatment_log_treatment_date_list):
                        print('\nFOUND LOG\n')
                        self.curr_log_file_path = self.log_folder_path + log_file
                        
                        print(treatment_log_row["Logged Treatment Start Time"])
                        treatment_time_local_log_df = self.get_treatment_local_data(self.curr_log_file_path, treatment_log_row["Logged Treatment Start Time"], treatment_log_row["Logged Treatment End Time"])
                        self.upload_pandas_dataframe(self.calc_treatment_data(treatment_log_row, treatment_time_local_log_df), self.treatment_log_table_id, self.airtable_api_key, self.log_base_id)
                        #self.upload_pandas_dataframe(self.get_row_turn_data_data(self.airtable_row_turn_log_df, treatment_log_row["Treatment ID"], treatment_time_local_log_df),self.row_turn_log_table_id, self.airtable_api_key, self.log_base_id)
                        break
    
    def get_treatment_local_data(self, log_file_path, treatment_start_time, treatment_end_time):
        #treatment_start_date_time = datetime.strptime(treatment_start_time, "%Y-%m-%dT%X.000Z").replace(tzinfo=pytz.UTC).astimezone(tz=None)
        #treatment_end_date_time = datetime.strptime(treatment_end_time, "%Y-%m-%dT%X.000Z").replace(tzinfo=pytz.UTC).astimezone(tz=None)

        treatment_start_date_time = datetime.strptime(treatment_start_time, "%m/%d/%Y %H:%M %p")
        treatment_end_date_time = datetime.strptime(treatment_end_time, "%m/%d/%Y %H:%M %p")
        local_log_df = pd.read_csv(log_file_path)
        treatment_time_local_log_df = local_log_df.loc[(local_log_df["Time"] >= treatment_start_date_time.timestamp()) & (local_log_df["Time"] <= treatment_end_date_time.timestamp())]
        return treatment_time_local_log_df


    def calc_treatment_data(self, treatment_log_row, treatment_time_local_log_df):
        #string manipulation with treatment start and end times
        
        
        treatment_log_row["GPS Head Fix"], treatment_log_row["GPS Tail Fix"] = self.get_gps_fix_percentage(treatment_time_local_log_df)
        treatment_log_row["Code Errors"] = treatment_time_local_log_df["rospy-error"].sum()
        treatment_log_row["Average Treatment Speed"] = treatment_time_local_log_df.loc[(treatment_time_local_log_df["Linear Velocity GPS"] > .05)]["Linear Velocity GPS"].mean()
        treatment_log_row["Time UV-C ON"] = (((treatment_time_local_log_df["All UVC Array Status"]==True).sum())/(treatment_time_local_log_df["All UVC Array Status"].sum()))*100
        treatment_log_row["Time in Manual Mode"] = (((treatment_time_local_log_df["Joystick Control"]==True).sum())/(treatment_time_local_log_df["Joystick Control"].sum()))*100
        treatment_log_row["# Waypoints Completed"] = treatment_time_local_log_df["Waypoint"].sum()
        #num_waypoints = treatment_time_local_log_df["Waypoint"].iloc[0]["num_waypoints"]
        #treatment_log_row["Waypoints Completed"] = treatment_time_local_log_df["Waypoint"].sum()/num_waypoints
        treatment_log_row["Map Used"] = rospy.get_param("waypoint_map")
        treatment_log_row["Robot Data Uploaded"] = True
        treatment_log_row = treatment_log_row.to_frame()
        print(treatment_log_row)
        print(treatment_log_row.transpose())
        return treatment_log_row.transpose()
    
    
    def get_row_turn_data(self, row_turn_df, treatment_id, treatment_time_local_log_df):
        pass
        #treatment_row_turn_log_df = row_turn_df.loc(row_turn_df["Treatment ID"] == treatment_id)
        #for record in treatment_row_turn_log_df:
        #    record
    
        



    def airtable_download(self, table, params_dict={}, api_key=None, base_id=None, record_id=None):
        """Makes a request to Airtable for all records from a single table.
            Returns data in dictionary format.
        Keyword Arguments:
        • table: set to table name
            ◦ see: https://support.airtable.com/hc/en-us/articles/360021333094#table
        • params_dict: desired parameters in dictionary format {parameter : value}
            ◦ example: {"maxRecords" : 20, "view" : "Grid view"}
            ◦ see "List Records" in API Documentation (airtable.com/api)
        • api_key: retrievable at https://airtable.com/account
            ◦ looks like "key●●●●●●●●●●●●●●"
        • base_id: retrievable at https://airtable.com/api for specific base
            ◦ looks like "app●●●●●●●●●●●●●●"
        • record_id: optional for single record lookups
            ◦ looks like "rec●●●●●●●●●●●●●●"
            """

        # Authorization Credentials
        if api_key is None:
            print("Enter Airtable API key. \n  *Find under Airtable Account Overview: https://airtable.com/account")
            api_key = input()
        headers = {"Authorization": "Bearer {}".format(api_key)}
        self.validate_airtable_kwargs(api_key, "API key", "key")

        # Locate Base
        if base_id is None:
            print("Enter Airtable Base ID. \n  *Find under Airtable API Documentation: https://airtable.com/api for specific base")
            base_id = input()
        url = 'https://api.airtable.com/v0/{}/'.format(base_id)
        path = url + table
        self.validate_airtable_kwargs(base_id, "Base ID", "app")

        # Validate Record ID
        if record_id is not None:
            self.validate_airtable_kwargs(record_id, "Record ID", "rec")

        # Format parameters for request
        constant_params = ()
        for parameter in params_dict:
            constant_params += ((parameter, params_dict[parameter]),)
        params = constant_params

        # Start with blank list of records
        airtable_records = []

        # Retrieve multiple records
        if record_id is None:
            run = True
            while run is True:
                response = requests.get(path, params=params, headers=headers)
                airtable_response = response.json()

                try:
                    airtable_records += (airtable_response['records'])
                except:
                    if 'error' in airtable_response:
                        self.identify_errors(airtable_response)
                        return airtable_response

                if 'offset' in airtable_response:
                    run = True
                    params = (('offset', airtable_response['offset']),) + constant_params
                else:
                    run = False

        # Retrieve single record
        if record_id is not None:
            if params_dict != {}:
                print("⚠️ Caution: parameters are redundant for single record lookups. Consider removing `params_dict` argument.")
            path = "{}/{}".format(path, record_id)
            response = requests.get(path, headers=headers)
            airtable_response = response.json()

            if 'error' in airtable_response:
                self.identify_errors(airtable_response)
                return airtable_response

            airtable_records = [airtable_response]

        return airtable_records


    def convert_to_dataframe(self, airtable_records):
        """Converts dictionary output from airtable_download() into a Pandas dataframe."""
        airtable_rows = []
        airtable_index = []
        for record in airtable_records:
            airtable_rows.append(record['fields'])
            airtable_index.append(record['id'])
        airtable_dataframe = pd.DataFrame(airtable_rows, index=airtable_index)
        return airtable_dataframe


    def create_field_matching_dict(self, airtable_records, value_field, key_field = None, swap_pairs = False):
        """Uses airtable_download() output to create a dictionary that matches field values from
        the same record together. Useful for keeping track of relational data.
        
        If second_field is `None`, then the dictionary pairs will be {<record id>:value_field}.
        Otherwise, the dictionary pairx will be {key_field:value_field}.
        If swap_pairs is True, then dictionary pairs will be {value_field:<record id>(or key_field)}.
        """
        airtable_dict = {}
        for airtable_record in airtable_records:
            if key_field == None:
                key = airtable_record['id']
            else:
                key = airtable_record['fields'].get(key_field)
            value = airtable_record['fields'].get(value_field)
            if swap_pairs:
                airtable_dict.update({key : value})
            else:
                airtable_dict.update({value : key})
        return airtable_dict


    def airtable_upload(self, table, upload_data, typecast = False, api_key = None, base_id = None, record_id = None):
        """Sends dictionary data to Airtable to add or update a record in a given table. 
            Returns new or updated record in dictionary format.
        
        Keyword arguments:
        • table: set to table name
            ◦ see: https://support.airtable.com/hc/en-us/articles/360021333094#table
        • upload_data: a dictionary of fields and corresponding values to upload in format {field : value}
            ◦ example: {"Fruit" : "Apple", "Quantity" : 20}
        • typecast: if set to true, Airtable will attempt "best-effort automatic data conversion from string values"
            • see: "Create Records" or "Update Records" in API Documentation, available at https://airtable.com/api for specific base
        • api_key: retrievable at https://airtable.com/account
            ◦ looks like "key●●●●●●●●●●●●●●"
        • base_id: retrievable at https://airtable.com/api for specific base
            ◦ looks like "app●●●●●●●●●●●●●●"
        • record_id: when included function will update specified record will be rather than creating a new record
            ◦ looks like "rec●●●●●●●●●●●●●●"
            """
        
        # Authorization Credentials
        if api_key == None:
            print("Enter Airtable API key. \n  *Find under Airtable Account Overview: https://airtable.com/account")
            api_key = input()
        headers = {"Authorization" : "Bearer {}".format(api_key),
                'Content-Type': 'application/json'}
        self.validate_airtable_kwargs(api_key, "API key", "key")

        # Locate Base
        if base_id == None:
            print("Enter Airtable Base ID. \n  *Find under Airtable API Documentation: https://airtable.com/api for specific base]")
            base_id = input()
        url = 'https://api.airtable.com/v0/{}/'.format(base_id)
        path = url + table
        self.validate_airtable_kwargs(base_id, "Base ID", "app")
        
        # Validate Record ID
        if record_id != None:
            self.validate_airtable_kwargs(record_id, "Record ID", "rec")
        
        # Validate upload_data
        if type(upload_data) != dict:
            print("❌ Error: `upload_data` is not a dictonary.")
            return

        # Create New Record
        if record_id == None:
            upload_dict = {"records": [{"fields" : upload_data}], "typecast" : typecast}
            upload_json = json.dumps(upload_dict)
            response = requests.post(path, data=upload_json, headers=headers)
            airtable_response = response.json()

        # Update Record
        if record_id != None:
            path = "{}/{}".format(path, record_id)
            upload_dict = {"fields" : upload_data, "typecast" : True}
            upload_json = json.dumps(upload_dict)
            response = requests.patch(path, data=upload_json, headers=headers)
            airtable_response = response.json()
        
        # Identify Errors
        if 'error' in airtable_response:
            self.identify_errors(airtable_response)
            
        return airtable_response


    def upload_pandas_dataframe(self, pandas_dataframe, table, api_key, base_id):
        """Uploads a Pandas dataframe to Airtable. If Pandas index values are Airtable Record IDs, will attempt to update 
            record. Otherwise, will create new records."""
       
        pandas_dicts = pandas_dataframe.to_dict(orient = "index")
        for pandas_dict in pandas_dicts:
            record_id = pandas_dict
            if self.validate_airtable_kwargs(str(record_id), "Record ID", "rec", print_messages=False) is False:
                record_id = None
            upload_data = pandas_dicts[pandas_dict]
            self.airtable_upload(table, upload_data, api_key=api_key, base_id=base_id, record_id=record_id)
        return


    # Troubleshooting Functions
    def validate_airtable_kwargs(self, kwarg, kwarg_name, prefix, char_length=17, print_messages=True):
        """Designed for use with airtable_download() and airtable_upload() functions.
            Checks `api_key`, `base_id` and `record_id` arguments to see if they conform to the expected Airtable API format.
            """
        valid_status = True
        if len(kwarg) != char_length:
            if print_messages is True:
                print("⚠️ Caution: {} not standard length. Make sure API key is {} characters long.".format(kwarg_name, char_length))
            valid_status = False
        if kwarg.startswith(prefix) is False:
            if print_messages is True:
                print("⚠️ Caution: {} doesn't start with `{}`.".format(kwarg_name, prefix))
            valid_status = False
        return valid_status


    def identify_errors(self, airtable_response):
        """Designed for use with airtable_download() and airtable_upload() functions.
            Prints error responses from the Airtable API in an easy-to-read format.
            """
        if 'error' in airtable_response:
            try:
                print('❌ {} error: "{}"'.format(airtable_response['error']['type'], airtable_response['error']['message']))
            except:
                print("❌ Error: {}".format(airtable_response['error']))
        return



if __name__ == "__main__":
    airtable_log_uploader = Airtable_Log_Uploader()
