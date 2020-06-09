#! /usr/bin/env python

"""
Create a node to start/shutdown a launch file.
"""

import rospy
import roslaunch

from std_srvs.srv import SetBool, SetBoolResponse

def init_roslaunch():
    global ROSLAUNCH_PARENT
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid=uuid)
    ROSLAUNCH_PARENT = roslaunch.parent.ROSLaunchParent(run_id=uuid, roslaunch_files=((LAUNCH_FILE,)))

def cb_launch(request):
    global IS_START, IS_RUNNING

    response = SetBoolResponse()
    response.success = True

    if request.data:
        if IS_RUNNING:
            response.success = False
            response.message = "is running"
        else:
            IS_START = True
            IS_RUNNING = True
            response.message = "start"
    else:
        IS_START = False
        IS_RUNNING = False
        ROSLAUNCH_PARENT.shutdown()
        rospy.loginfo("launch_node stop : {}".format(LAUNCH_FILE))
        init_roslaunch()
        response.message = "shutdown"

    return response

if __name__ == "__main__":

    rospy.init_node(name='switch', anonymous=False)

    # -- Get parameters
    LAUNCH_FILE = rospy.get_param(param_name="~launch_file")

    # -- Node function
    rospy.Service(name="~turn_on", service_class=SetBool, handler=cb_launch)

    # -- For running ROSLaunchParent in main process
    IS_START = False
    IS_RUNNING = False
    ROSLAUNCH_PARENT = None
    init_roslaunch()
    rate = rospy.Rate(0.5)

    while not rospy.is_shutdown():
        if IS_START:
            rospy.loginfo("switch start : {}".format(LAUNCH_FILE))
            ROSLAUNCH_PARENT.start()
            IS_START = False
        rate.sleep()
