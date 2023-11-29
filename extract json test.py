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

# Iterate through the points in the JSON map
in_row = False
row_start_point = None
row_start_time = None

for point in data['points']:
    if point['datum']:
        row_start_point = point
        in_row = True
        row_start_time = point['timestamp']

    if point['treatment_area']:
        if in_row:
            treatment_sections.append(MapSection('row', row_start_point, point, point['timestamp'] - row_start_time))
            in_row = False

    # Add similar logic for turns
    # ...

# You can print or analyze the sections
for section in treatment_sections:
    print(f"Type: {section.section_type}, Start Point: {section.start_point}, End Point: {section.end_point}, Duration: {section.duration}")
