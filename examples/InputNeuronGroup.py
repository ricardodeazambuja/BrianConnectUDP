'''
Example of a spike generator (only outputs spikes)

In this example spikes are generated and sent through UDP packages. At the end of the simulation a raster plot of the 
spikes is created.

'''

from brian import *
import numpy

from brian_multiprocess_udp import BrianConnectUDP

def main_NeuronGroup(input_Neuron_Group, simulation_clock):
    print "main_NeuronGroup!" #DEBUG!

    simclock = simulation_clock

    delta_t=5
    spiketimes = [(i, delta_t*ms) for i in range(5)]+[(i, delta_t*ms) for i in range(10,15)]+[(i, delta_t*ms) for i in range(30,35)]+[(i, delta_t*ms) for i in range(40,45)]
   
    SpikesOut = SpikeGeneratorGroup(45, spiketimes, period=10*ms, clock=simclock) # the maximum clock of the input spikes is limited here (period)

    MSpkOut=SpikeMonitor(SpikesOut) # Spikes sent by UDP

    return ([SpikesOut],[],[MSpkOut])

def post_simulation_function(input_NG, simulation_NG, simulation_SYN, simulation_MN):
    """
    input_NG: the neuron group that receives the input spikes
    simulation_NG: the neuron groups list passed to the system by the user function (main_NeuronGroup)
    simulation_SYN: the synapses list passed to the system by the user function (main_NeuronGroup)
    simulation_MN: the monitors list passed to the system by the user function (main_NeuronGroup)

    This way it is possible to plot, save or do whatever you want with these objects after the end of the simulation!
    """
    figure()
    raster_plot(simulation_MN[0])
    title("Spikes Sent by UDP")
    show(block=True) 

if __name__=="__main__":

    my_simulation = BrianConnectUDP(main_NeuronGroup, NumOfNeuronsOutput=45, post_simulation_function=post_simulation_function,
        UDP_IPO=["127.0.0.1"], UDP_PORTO=[20202], simclock_dt=5, TotalSimulationTime=20000)    

    # my_simulation = BrainConnectUDP(main_NeuronGroup, NumOfNeuronsOutput=45, post_simulation_function=post_simulation_function,
    #     UDP_IPO=["192.168.1.123"], UDP_PORTO=[20202], simclock_dt=5, TotalSimulationTime=10000)        
