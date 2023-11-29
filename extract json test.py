import json


# Initialize variables
start_data = []
with open('Appended.json', 'r') as file:
    data = json.load(file)
# Access 'treatment_area' values
for point in data.get('points', []):
    treatment_area_value = point.get('treatment_area')

    if treatment_area_value is not None:
        if not start_data and treatment_area_value == False:
            # Found the first 'false' point, start capturing data
            start_data.append(treatment_area_value)

        if start_data:
            # Continue capturing treatment_area values until the first 'true' point
            start_data.append(treatment_area_value)

        if start_data and treatment_area_value == True:
            # Found the first 'true' treatment area point, stop capturing data
            break

# Print the captured 'start' data
print(start_data)


Output:

[False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, False, True]


Need to repeat the process for each row
