import json

# Assuming json_data contains your JSON data
data = json.loads(json_data)

# Initialize variables
start_data = []
row_data = []
capturing_start = True  # Flag to determine whether to capture 'start' or 'row' data

# Access 'treatment_area' values
for point in data.get('points', []):
    treatment_area_value = point.get('treatment_area')

    if capturing_start:
        if treatment_area_value is not None and treatment_area_value is False:
            start_data.append(treatment_area_value)
        elif start_data:
            capturing_start = False

    else:
        if treatment_area_value is not None and treatment_area_value is True:
            row_data.append(treatment_area_value)
        elif row_data:
            break  # Stop capturing 'row' data when the next 'false' is encountered

# Print the captured 'start' data and 'row' data
print("Start Data:", start_data)
print("Row Data:", row_data)
