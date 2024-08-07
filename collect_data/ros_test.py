import rospy
import os
import logging
import h5py

logging.basicConfig(level=logging.DEBUG)

def main():
    print("Starting the node")
    a = os.getpid()
    print(f"Process ID before rospy.init_node: {a}")
    try:
        rospy.init_node('minimal_node', anonymous=True, log_level=rospy.DEBUG)
        print("1 - Node Initialized")
        resolved_node_name = rospy.get_name()
        rospy.loginfo("init_node, name[%s], pid[%s]", resolved_node_name, os.getpid())
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print(f"ROSInterruptException occurred: {e}")
    except Exception as e:
        print(f"Exception occurred: {e}")

if __name__ == '__main__':
    print(h5py.__version__)
    main()
    print(1)
