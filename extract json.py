import json

def extract_treatment_area(json_data):
    treatment_areas = []

    # Iterate through the points in the JSON data
    for point in json_data.get('points', []):
        # Check if the necessary nested keys exist
        if 'head' in point and 'position' in point['head']:
            position = point['head']['position']
            
            # Check if 'treatment_area' key exists in the position dictionary
            if 'treatment_area' in position and isinstance(position['treatment_area'], bool):
                treatment_areas.append(position['treatment_area'])

    return treatment_areas

def main():
    # Load JSON data from the file
    with open('appended.json', 'r') as file:
        json_data = json.load(file)

    # Extract treatment_area booleans
    treatment_areas = extract_treatment_area(json_data)

    # Print the result
    print("Treatment Areas:", treatment_areas)

if __name__ == "__main__":
    main()
