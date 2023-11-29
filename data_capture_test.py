def calculate_linear_feet_driven(treatment_data):
    # Calculate the total linear distance traveled for the treatment
    total_distance = 0
    for index, row in treatment_data.iterrows():
        if row['Linear Velocity GPS'] > 0.05:
            distance_in_timestamp = row['Linear Velocity GPS'] * (row['End Timestamp'] - row['Start Timestamp'])
            total_distance += distance_in_timestamp

    # Convert distance from meters to feet
    linear_feet_driven = total_distance * 3.28084

    return linear_feet_driven

def calculate_percent_manual_auto(treatment_data):
    # Identify timestamps for transitions between manual and auto modes
    manual_to_auto_transitions = []
    auto_to_manual_transitions = []

    for index, row in treatment_data.iterrows():
        if index > 0 and row['Control Mode'] == 'Auto' and treatment_data.loc[index - 1, 'Control Mode'] == 'Manual':
            manual_to_auto_transitions.append(row['Start Timestamp'])
        elif index > 0 and row['Control Mode'] == 'Manual' and treatment_data.loc[index - 1, 'Control Mode'] == 'Auto':
            auto_to_manual_transitions.append(row['Start Timestamp'])

    # Calculate total time in manual and auto modes
    manual_time = 0
    auto_time = 0

    if len(manual_to_auto_transitions) > 0:
        for i in range(1, len(manual_to_auto_transitions)):
            manual_time += manual_to_auto_transitions[i] - manual_to_auto_transitions[i - 1]

    if len(auto_to_manual_transitions) > 0:
        for i in range(1, len(auto_to_manual_transitions)):
            auto_time += auto_to_manual_transitions[i] - auto_to_manual_transitions[i - 1]

    # Calculate percent manual and auto operation
    percent_manual = (manual_time / (manual_time + auto_time)) * 100
    percent_auto = (auto_time / (manual_time + auto_time)) * 100

    return percent_manual, percent_auto

def calculate_mtba(assist_events):
    # Calculate time intervals between assist events
    time_intervals_between_assists = []
    for i in range(1, len(assist_events)):
        time_interval = assist_events[i] - assist_events[i - 1]
        time_intervals_between_assists.append(time_interval)

    # Calculate mean time between assists (MTBA)
    mtba = sum(time_intervals_between_assists) / len(time_intervals_between_assists)

    return mtba

def main():
    # Read treatment data from CSV file
    treatment_data = pd.read_csv('treatment_data.csv')

    # Calculate linear feet driven
    linear_feet_driven = calculate_linear_feet_driven(treatment_data)

    # Calculate percent manual and auto
    percent_manual, percent_auto = calculate_percent_manual_auto(treatment_data)

    # Print results
    print("Linear Feet Driven:", linear_feet_driven)
    print("Percent Manual:", percent_manual)
    print("Percent Auto:", percent_auto)

if __name__ == '__main__':
    main()
