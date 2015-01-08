
def random_init():

    import numpy
    import time

    import rospy
    import baxter_interface

    # Initialization of the ROS node.
    print("Initializing Baxter node... ")
    rospy.init_node("random_joint_position", disable_signals=True) # This node name must be UNIQUE!
    # the option "disable_signals=True" is important to make ROS ignore the signals from my program.

    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    init_state = rs.state().enabled

    # def clean_shutdown():
    #     print("\nExiting example...")
    #     if not init_state:
    #         print("Disabling robot...")
    #         rs.disable()

    # rospy.on_shutdown(clean_shutdown)

    if not init_state:
        print("Enabling robot... ")
        rs.enable()

    # Initializes the left arm
    left = baxter_interface.Limb('left')
    time.sleep(1) #gives some time to ROS...

    # Gets the joint names used inside the joint_command
    lj = left.joint_names()
    # lj[0] => s0
    # lj[1] => s1
    # lj[2] => e0
    # lj[3] => e1
    # lj[4] => w0
    # lj[5] => w1
    # lj[6] => w2

    joint_names = lj

    cmd = dict(zip(joint_names, numpy.random.uniform(-numpy.pi,numpy.pi,7)))
    left.set_joint_positions( cmd )
    time.sleep(1)

    # Prints the current endpoint position.
    print "Baxter Random Init:"
    print cmd



if __name__=="__main__":
    random_init()
