import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import yaml
from time import sleep
y = yaml.safe_load(open("route.yaml", "r"))
    


def main():
    rospy.init_node('set_pose')

    for pose in y['poses']:

        print(pose)
        state_msg = ModelState()
        state_msg.model_name = 'bibo'
        state_msg.pose.position.x = pose['pose']['position']['x']
        state_msg.pose.position.y = pose['pose']['position']['y']
        state_msg.pose.position.z = pose['pose']['position']['z'] + 0.2
        state_msg.pose.orientation.x = pose['pose']['orientation']['x']
        state_msg.pose.orientation.y = pose['pose']['orientation']['y']
        state_msg.pose.orientation.z = pose['pose']['orientation']['z']
        state_msg.pose.orientation.w = pose['pose']['orientation']['w']
        
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException:
            print ("Service call failed: %s" % e)
        try:
            sleep(5)
        except:
            exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
