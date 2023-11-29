import json

# Specify the path to your JSON file
json_file_path = 'path/to/your/file.json'

# Initialize variables
all_data = []
capturing_start = True  # Flag to determine whether to capture 'start' or 'row' data
start_data = []  # Initialize 'start_data' outside the loop

# Open the JSON file using 'with open()' method
with open(json_file_path, 'r') as file:
    data = json.load(file)

    # Access 'treatment_area' values for each set
    for point in data.get('points', []):
        treatment_area_value = point.get('treatment_area')

        if capturing_start:
            if treatment_area_value is not None and treatment_area_value is False:
                start_data = [treatment_area_value]
                capturing_start = False
        else:
            if treatment_area_value is not None and treatment_area_value is True:
                row_data = [treatment_area_value]
                capturing_start = True
                all_data.append({"start": start_data, "row": row_data})
            elif row_data:
                row_data.append(treatment_area_value)

# Print all the captured data
for idx, data_set in enumerate(all_data, start=1):
    print(f"Set {idx} - Start Data:", data_set["start"])
    print(f"Set {idx} - Row Data:", data_set["row"])
    print()
