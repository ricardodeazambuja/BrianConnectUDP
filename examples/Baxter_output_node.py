'''
This is the output node that send spikes according to the end point position

It should send this information to the reward calculation node and to the liquid (feedback)

'''

import argparse

from brian_multiprocess_udp import BrianConnectUDP


def from_baxter_to_spikes(spikes_pipe_in, spikes_pipe_out):
    """
    This function substitutes the run_brian_simulation and converts the received spikes into Baxter joint angles.
    """

    import numpy
    import select
    import time

    import rospy
    import baxter_interface


    #
    # Initializes the dictionary position translator!
    my_randseed = 123456
    numpy.random.seed(my_randseed) #with this seed, the generation always repeats!
    NoSN = 10 # Number of simultaneous spike
    NoTicks = 2301

    unique_set=set() # Sets accept only unique elements, so any duplicate is erased (or not added)
                     # Then, using a set, is possible to easily generate lists of unique elements.

    # o = numpy.array([0]*Number_of_Neurons_Output,dtype=numpy.uint8)
    i=0
    while len(unique_set)<NoTicks: # My final set is going to be made of NoTicks spike trains (to code numbers from 0 to NoTicks-1)
        i+=1
        unique_set.add(numpy.array(numpy.random.permutation(range(Number_of_Neurons_Output))[0:NoSN],dtype=numpy.uint8).tostring())
        # o[numpy.array(numpy.random.permutation(range(Number_of_Neurons_Output))[0:NoSN],dtype=numpy.uint8)]=[1]*NoSN
        # unique_set.add(o.tostring())
        # o.fill(0)

    # while len(unique_set)<2301: # My final set is going to be made of 2301 spike trains (to code numbers from -1000 to 1300)
    #     unique_set.add(numpy.array(numpy.random.permutation(range(Number_of_Neurons_Output))[0:NoSN],dtype=numpy.uint8).tostring())
        # Shuffles a list with numbers from 0 to Number_of_Neurons_Output, gets the first NoSN items and converts to string.
        # Because only the first NoSN items are used, it means the spike train will be made of NoSN spikes out of Number_of_Neurons_Output neurons.
        # The convertion to string is necessary because a set cannot accetp a list or numpy.array.

    # The Brian simulator outputs spike trains as a list of the indexes of the neurons who spiked.
    # Therefore I also need to generate lists of indexes instead of full spike trains with ones and zeros.

    unique_set_indexes=[numpy.fromstring(i, dtype=numpy.uint8) for i in unique_set]
    # Here the lists are recovered from the strings inside the set.

    unique_set_spike_trains=numpy.zeros((len(unique_set_indexes),Number_of_Neurons_Output), dtype=numpy.uint8)
    # A matrix of numpy.array full of zeros is generated here. Above were generated only indexes, so this
    # matrix will be used to set the positions where a spike should occur form 0 to 1.


    for i,j in zip(unique_set_indexes,range(len(unique_set_indexes))):
        unique_set_spike_trains[j][i]=1
        # Here the matrix has the positions set to one according to the indexes.

    input_dict=dict(zip([i.tostring() for i in unique_set_spike_trains],range(-1000,1301))) 
    # Receives a spike train converted tostring and returns a number (from -1000 to 1300)

    output_dict=dict(zip(range(-1000,1301),unique_set_indexes)) 
    # Receives a number (from -1000 to 1300) and returns the spike indexes

    #
    #



    # Initialization of the ROS node.
    print("Initializing Baxter node... ")
    rospy.init_node("end_point_to_spikes", disable_signals=True) # This node name must be UNIQUE!
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

    while True:
        # Prints the current endpoint position.
        current_position=(left.endpoint_pose()['position'].x,left.endpoint_pose()['position'].y,left.endpoint_pose()['position'].z)
        
        X=output_dict[int(current_position[0]*1000)]
        Y=output_dict[int(current_position[1]*1000)]
        Z=output_dict[int(current_position[2]*1000)]

        spikes_pipe_out.send(numpy.concatenate((X,Y,Z)))

        print "Baxter Output Node"
        print "Spikes Indexes - X:%s Y:%s Z:%s" % (X,Y,Z)
        print "Position - X:%s Y:%s Z:%s" % (int(current_position[0]*1000),int(current_position[1]*1000),int(current_position[2]*1000))

        time.sleep(outputclock_dt/1000.0) # This is the only thing to prevent the while loop from going crazy.



if __name__=="__main__":


    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Sets up and launch the Baxter OUTPUT node.")

    parser.add_argument("--output_neurons", help="The total number of OUTPUT neurons.", type=int, default=30*3)
    parser.add_argument("--output_clock", help="An integer number used as the OUTPUT clock (mS)", type=int, default=100)

    parser.add_argument("--output_addr", help="Output addresses as a list of tuples with [(\"IP\", PORT, number of neurons)].", type=list, default=[("192.168.1.123", 11112)])
    parser.add_argument("--brian_addr", help="A number from 0 to 255 used to identify the node.", type=int, default=201)
    parser.add_argument("--ttime", help="Total time of the simulation.", type=int, required=True)

    
    
    args=parser.parse_args()


    outputclock_dt = args.output_clock

    Number_of_Neurons_Output = args.output_neurons


    my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsOutput=Number_of_Neurons_Output,
        output_addresses=args.output_addr, TotalSimulationTime=args.ttime, brian_bypass=from_baxter_to_spikes, brian_address=args.brian_addr)
