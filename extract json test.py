import json

# Load JSON data from the file
with open('Appended.json', 'r') as file:
    data = json.load(file)

# Initialize a variable to track whether 'start' should be captured
capture_start = False
start_data = []

# Access 'treatment_area' values
for point in data.get('points', []):
    treatment_area_value = point.get('treatment_area')

    if treatment_area_value is not None:
        if not capture_start and treatment_area_value == False:
            # Found the first 'false' point, start capturing data
            capture_start = True

        if capture_start:
            # Capture the data
            start_data.append(point)

        if capture_start and treatment_area_value == True:
            # Found the first 'true' treatment area point, stop capturing data
            break

# Print the captured 'start' data
print(json.dumps(start_data, indent=2))
