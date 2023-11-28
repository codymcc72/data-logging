import json

# Load JSON data from the file
with open('Appended.json', 'r') as file:
    data = json.load(file)

# Access 'treatment_area' values
for point in data.get('points', []):
    treatment_area_value = point.get('treatment_area')
    if treatment_area_value is not None:
        print(treatment_area_value)
    else:
        print("Treatment area not found for a point.")
