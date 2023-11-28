import rospy
from std_msgs.msg import String  # Import the message type you want to subscribe to

def callback(data):
    rospy.loginfo("Received message: %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)

    # Specify the topic you want to subscribe to
    topic_name = '/your/ros/topic'  # Replace with the actual topic name

    # Specify the message type of the topic
    message_type = String  # Replace with the actual message type

    rospy.Subscriber(topic_name, message_type, callback)

    # Spin to keep the script alive and receive messages
    rospy.spin()

if __name__ == '__main__':
    listener()
