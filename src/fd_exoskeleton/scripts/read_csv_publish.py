#!/usr/bin/env python3

import rospy
import csv
from std_msgs.msg import Float64MultiArray
from fd_exoskeleton.msg import CmdMessage
# Global variable to store CSV data
csv_data = []
pub = None  # Global publisher

# Function to read CSV file and store its data in a list
def read_csv_file():
    global csv_data
    try:
        with open('csv/motor.csv', mode='r') as file:
            reader = csv.reader(file)
            header = next(reader)  # Skip header line (if any)
            for row in reader:
                # Convert each row's values to float and store them as a 9-dimensional array
                if len(row) == 9:  # Ensure each row has 9 columns
                    csv_data.append([float(value) for value in row])
                else:
                    rospy.logwarn(f"Skipping invalid row: {row}")
        rospy.loginfo(f"Successfully read {len(csv_data)} rows from the CSV file.")
    except Exception as e:
        rospy.logerr(f"Failed to read CSV file: {e}")

        
def publish_csv_data():
    global csv_data
    for index, row in enumerate(csv_data):
        # Publish the 9 values as a Float64MultiArray
        output = Float64MultiArray()
        output.data = row
        pub.publish(output)  # Publish the data
        rospy.loginfo(f"Published row {index}: {row}")
        
        # Sleep for 0.5 seconds before publishing the next row
        rospy.sleep(0.5)

def callback(data):
    rospy.loginfo(f'cmd: {data.cmd} value{data.value}')
    if "fall" in data.cmd:
        # Immediately publish all rows from CSV
        publish_csv_data()

# Main function to initialize ROS node and publish data
def main():
    global pub
    # Initialize the node
    rospy.init_node('csv_publisher', anonymous=True)

    # Create the publisher once
    pub = rospy.Publisher('csv_output', Float64MultiArray, queue_size=10)

    rospy.Subscriber('cmd', CmdMessage, callback)

    read_csv_file()
 
    # Keep the node running, even though we're not using subscribers
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
