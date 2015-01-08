'''

Generates the user input spikes to the liquid

'''

import brian_no_units # Speeds up Brian by ignoring the units

from brian import *

from brian_multiprocess_udp import BrianConnectUDP

import numpy

import time

outputclock_dt = 100 #in milliseconds

Number_of_Neurons_Output = 30*3

current_position = (0.445,0.433,0.113)
# current_position = (0.811,-0.011,0.349)
# current_position = (0.295,0.835,0.597) #End point goal position (X,Y,Z) in metres


def user_input_to_spikes(spikes_pipe_in, spikes_pipe_out):
    """
    This function substitutes the run_brian_simulation and converts the user input into spikes
    """

    print "User Input!" #DEBUG!

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



    while True:
        
        X=output_dict[int(current_position[0]*1000)]
        Y=output_dict[int(current_position[1]*1000)]
        Z=output_dict[int(current_position[2]*1000)]

        spikes_pipe_out.send(numpy.concatenate((X,Y,Z)))

        print "Input Neuron Group"
        print "Spikes Indexes - X:%s Y:%s Z:%s" % (X,Y,Z)
        print "Position - X:%s Y:%s Z:%s" % current_position
        
        time.sleep(outputclock_dt/1000.0)


    return ([SpikesOut],[],[])


if __name__=="__main__":

    my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsOutput=Number_of_Neurons_Output,
        output_addresses=[("127.0.0.1", 11111)], TotalSimulationTime=600000, brian_bypass=user_input_to_spikes, brian_address=0)

    # my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsOutput=Number_of_Neurons_Output,
    #     output_addresses=[("127.0.0.1", 30303)], TotalSimulationTime=100000, brian_bypass=user_input_to_spikes, brian_address=0)    
