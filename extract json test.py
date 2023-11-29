import json


class MapSection:
    def __init__(self, section_type, start_point, end_point, duration):
        self.section_type = section_type
        self.start_point = start_point
        self.end_point = end_point
        self.duration = duration

# Load JSON data
with open('appended.json', 'r') as file:
    data = json.load(file)

# Initialize lists to store different sections
start_sections = []
treatment_sections = []
turn_sections = []

# Initialize variables to store 'datum' information
datum_info = None

# Iterate through the points in the JSON map
in_row = False
row_start_point = None
row_start_time = None

for point in data.get('points', []):
    datum_value = point.get('datum')
    if datum_value:
        datum_info = datum_value

    treatment_area_value = point.get('treatment_area')
    if treatment_area_value:
        if in_row:
            treatment_sections.append(MapSection('row', row_start_point, point, point['timestamp'] - row_start_time))
            in_row = False

# Access 'datum' information after the loop
if datum_info:
    print("Datum Information:")
    print(f"Latitude: {datum_info.get('latitude')}, Longitude: {datum_info.get('longitude')}, Altitude: {datum_info.get('altitude')}")

# You can print or analyze the sections
for section in treatment_sections:
    print(f"Type: {section.section_type}, Start Point: {section.start_point}, End Point: {section.end_point}, Duration: {section.duration}")
