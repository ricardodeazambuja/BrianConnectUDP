'''
Generates the reward spikes!



'''

import argparse

from brian_multiprocess_udp import BrianConnectUDP


def reward_generator(spikes_pipe_in, spikes_pipe_out):
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
    rospy.init_node("reward_node", disable_signals=True) # This node name must be UNIQUE!
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

    old_position=(left.endpoint_pose()['position'].x,left.endpoint_pose()['position'].y,left.endpoint_pose()['position'].z)
    time.sleep(outputclock_dt/1000.0)

    distances = []
    counter = 0
    sleep_time=outputclock_dt
    old_dist = current_position = av_dist = None
    max_error = 0.01 # in metres
    while True:
        if current_position != None:
			old_position = current_position

        # Prints the current endpoint position.
        current_position=(left.endpoint_pose()['position'].x,left.endpoint_pose()['position'].y,left.endpoint_pose()['position'].z)

        if current_position==old_position:
            time.sleep(sleep_time/1000.0)            
            continue

        
        current_dist=(goal_position[0]-current_position[0])**2+(goal_position[1]-current_position[1])**2+(goal_position[2]-current_position[2])**2

        distances.append(current_dist)

        if len(distances) == 1:
            old_dist=current_dist
        
        if len(distances) == 10:
            av_dist = sum(distances)/len(distances)
            distances = []

            if int(100*av_dist)<int(100*old_dist) or av_dist<=max_error:
                spikes_pipe_out.send([1]) # YES, reward!
		                                  # Sinalizes that the neuron number 1 (second) spiked.
                sleep_time=outputclock_dt/5
                #print "REWARD!" #DEBUG!
            elif av_dist>max_error:
                spikes_pipe_out.send([0]) # NO, punishment!
		                                  # Sinalizes that the neuron number 0 (first) spiked.
                sleep_time=outputclock_dt
                #print "PUNISHMENT!" #DEBUG!
        
        if old_dist != None and av_dist != None:
            print "Baxter Reward Node"
            print "Current/Old distance/Average:%s/%s/%s " % (int(1000*current_dist), int(1000*old_dist), int(1000*av_dist))

        time.sleep(sleep_time/1000.0)



if __name__=="__main__":

    current_position = (0.445,0.433,0.113)
    
    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Sets up and launch the REWARD node.")

    parser.add_argument("--output_neurons", help="The total number of OUTPUT neurons.", type=int, default=2)
    parser.add_argument("--output_clock", help="An integer number used as the OUTPUT clock (mS)", type=int, default=100)
    parser.add_argument("--goal_pos", help="An tuple (X,Y,Z) used as the GOAL position", type=tuple, default=current_position)

    parser.add_argument("--output_addr", help="Output addresses as a list of tuples with [(\"IP\", PORT, number of neurons)].", type=list, default=[("192.168.1.123", 22223)])
    parser.add_argument("--brian_addr", help="A number from 0 to 255 used to identify the node.", type=int, default=202)
    parser.add_argument("--ttime", help="Total time of the simulation.", type=int, required=True)
    
    
    args=parser.parse_args()

    outputclock_dt = args.output_clock

    Number_of_Neurons_Output = args.output_neurons

    goal_position = args.goal_pos

    my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsOutput=Number_of_Neurons_Output,
        output_addresses=args.output_addr, TotalSimulationTime=args.ttime, brian_bypass=reward_generator, brian_address=args.brian_addr)
