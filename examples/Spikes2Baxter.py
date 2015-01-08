'''
This is the input node that receives spikes and converts to Baxter joint angles.

'''

import argparse

from brian_multiprocess_udp import BrianConnectUDP



def from_spikes_to_baxter(spikes_pipe_in, spikes_pipe_out):
    """
    This function substitutes the run_brian_simulation and converts the received spikes into Baxter joint angles.
    """

    import numpy
    import select
    import time

    import rospy
    import baxter_interface

    # Initialization of the ROS node.
    print("Initializing Baxter node... ")
    rospy.init_node("spikes_to_joint_position", disable_signals=True) # This node name must be UNIQUE!
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

    def converts_spikes_into_joints(spike_numpy_array, limb, step_up, step_down):
        """
        Receives a numpy.array with 4 elements. In case a spike is received at:
        Index 0 => increase S0
        Index 1 => decrease S0
        Index 2 => increase S1
        Index 3 => decrease S1

        spike_numpy_array => the numpy array received directly from my system (a numpy array with 1's and 0's) and the 
                             same length as NumOfNeuronsInput.

        joint_angles => list of the angles following the same order as joint_names

        step_up => amount to be increased at the angle if a spike is received

        step_down => amount to be decreased at the angle if a spike is received

        returns a dict with all the joint angles values
        """
        joint_angles = limb.joint_angles() #gets the actual joint angles

        joint_names = ['left_s0','left_s1','left_e0','left_e1','left_w0','left_w1','left_w2']

        # movement = spike_numpy_array*step_up - (spike_numpy_array*(-1)+1)*step_down
        move_S0 = 0
        move_S1 = 0        

        if spike_numpy_array[0]==1:
            move_S0=step_up

        if spike_numpy_array[1]==1:
            move_S0=-step_down

        if spike_numpy_array[2]==1:
            move_S1=step_up

        if spike_numpy_array[3]==1:
            move_S1=-step_down

        movement = numpy.array([move_S0, move_S1] + [0]*5)

        # Creates a dictionary with the joint names and the necessary steps (up or down)
        final_step = dict(zip(joint_names, movement))

        # Returns the actual joint angles summed by the steps according to the spike_numpy_array received
        return {key:value+final_step[key] for key,value in joint_angles.iteritems()}

    print "Node started!"

    while True:

        if select.select([spikes_pipe_in],[],[]):

            # Receives the spike train from the pipe, converts according to the converts_spikes_into_joints function 
            # and them apply the dictionary with the joint names and angles to Baxter using the set_joint_positions method
            # from the limb object (left).
            #print "spikes in!"
            cmd = converts_spikes_into_joints(spikes_pipe_in.recv(), left, 0.25, 0.25)
            #print "set joints!", 
            left.set_joint_positions( cmd )
            #print "Done!"
            # Prints the current endpoint position.
            print "Baxter Input Node"
            print cmd



if __name__=="__main__":

    #my_randseed=int(''.join(map(str,numpy.random.randint(0,9,15)))) # creates a long integer converted random seed, but it is a crazy dumb code :)

    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Sets up and launch the Baxter OUTPUT node.")

    parser.add_argument("--input_neurons", help="The total number of INPUT neurons.", type=int, default=4)
    parser.add_argument("--input_addr", help="Input addresses as a tuple with (\"IP\", PORT).", type=list, default=("192.168.1.111", 33333))
    parser.add_argument("--input_clock", help="An integer number used as the INPUT clock (mS)", type=int, default=100)

    parser.add_argument("--brian_addr", help="A number from 0 to 255 used to identify the node.", type=int, default=200)
    parser.add_argument("--ttime", help="Total time of the simulation.", type=int, required=True)

    
    
    args=parser.parse_args()


    my_inputclock_dt = args.input_clock

    Number_of_Neurons_Input = args.input_neurons

    my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsInput=Number_of_Neurons_Input,
        input_addresses=[(args.input_addr[0], args.input_addr[1], Number_of_Neurons_Input)],
        inputclock_dt=my_inputclock_dt, TotalSimulationTime=args.ttime, brian_bypass=from_spikes_to_baxter, brian_address=args.brian_addr)
