'''
Example of a spike generator (only outputs spikes)

In this example spikes are generated and sent through UDP packages. At the end of the simulation a raster plot of the 
spikes is created.

'''
import brian_no_units # Speeds up Brian by ignoring the units

from brian import *
import numpy

from brian_multiprocess_udp import BrianConnectUDP

number_of_neurons_total = 7
number_of_neurons_spiking = 3


def main_NeuronGroup(input_Neuron_Group, simulation_clock):
    print "main_NeuronGroup!" #DEBUG!

    simclock = simulation_clock

    delta_t=5
    

    def nextspike():
        # nexttime = numpy.random.uniform(50E-3,100E-3)
        nexttime = 0        
        random_list=range(number_of_neurons_total)
        while True:
            numpy.random.shuffle(random_list)
            # for i in random_list[0:numpy.random.randint(1,number_of_neurons_total+1)]:
            for i in random_list[0:number_of_neurons_spiking]:    
                yield (i,nexttime)
            nexttime = nexttime + 20E-3

   
    SpikesOut = SpikeGeneratorGroup(number_of_neurons_total, nextspike, clock=simclock) # the maximum clock of the input spikes is limited here (period)



    return ([SpikesOut],[],[])


if __name__=="__main__":

    my_simulation = BrianConnectUDP(main_NeuronGroup, NumOfNeuronsOutput=number_of_neurons_total,
        output_addresses=[("127.0.0.1", 18181)], simclock_dt=5, TotalSimulationTime=120000, brian_address=0)
