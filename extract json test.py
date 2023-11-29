import json

# Initialize variables
all_rows_data = []

with open('Appended.json', 'r') as file:
    data = json.load(file)

# Access 'treatment_area' values
for point in data.get('points', []):
    treatment_area_value = point.get('treatment_area')

    if treatment_area_value is not None:
        if treatment_area_value is False:
            # Found the start of a new row, initialize variables
            start_data = [treatment_area_value]
            row_data = []
        elif start_data:
            # Continue capturing treatment_area values until the next 'True' point
            start_data.append(treatment_area_value)
            if treatment_area_value is True:
                row_data = start_data.copy()
                all_rows_data.append(row_data)
                start_data = []

# Print the captured data for each row
for idx, row_data in enumerate(all_rows_data, start=1):
    print(f"Row {idx} Data:", row_data)
