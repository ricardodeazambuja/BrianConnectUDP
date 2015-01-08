'''
Example of a spike receptor (only receives spikes)

In this example spikes are received and processed creating a raster plot at the end of the simulation.

'''

from brian import *
import numpy

from brian_multiprocess_udp_mono import BrianConnectUDP

# The main function with the NeuronGroup(s) and Synapse(s) must be named "main_NeuronGroup".
# It will receive two objects: input_Neuron_Group and the simulation_clock. The input_Neuron_Group
# will supply the input spikes to the network. The size of the spike train received equals NumOfNeuronsInput.
# The size of the output spike train equals NumOfNeuronsOutput and must be the same size of the NeuronGroup who is
# going to interface with the rest of the system to send spikes.
# The function must return all the NeuronGroup objects and all the Synapse objects this way:
# ([list of all NeuronGroups],[list of all Synapses])
# and the FIRST (index 0) NeuronGroup of the list MUST be the one where the OUTPUT spikes will be taken by the simulation.
# 
# Here is also possible to use "dummy" NeuronGroups only to receive and/or send spikes.

def main_NeuronGroup(input_Neuron_Group, simulation_clock):
    print "main_NeuronGroup!" #DEBUG!

    simclock = simulation_clock

    Nr=NeuronGroup(45, model='v:1', reset=0, threshold=0.5, clock=simclock)
    Nr.v=0

    # SYNAPSES BETWEEN REAL NEURON NETWORK AND THE INPUT
    Syn_iNG_Nr=Synapses(input_Neuron_Group, Nr, model='w:1', pre='v+=w', clock=simclock)

    Syn_iNG_Nr[:,:]='i==j'

    print "Total Number of Synapses:", len(Syn_iNG_Nr) #DEBUG!

    Syn_iNG_Nr.w=1

    MExt=SpikeMonitor(Nr) # Spikes sent by UDP

    Mdummy=SpikeMonitor(input_Neuron_Group) # Spikes received by UDP

    return ([Nr],[Syn_iNG_Nr],[MExt,Mdummy])

def post_simulation_function(input_NG, simulation_NG, simulation_SYN, simulation_MN):
    """
    input_NG: the neuron group that receives the input spikes
    simulation_NG: the neuron groups list passed to the system by the user function (main_NeuronGroup)
    simulation_SYN: the synapses list passed to the system by the user function (main_NeuronGroup)
    simulation_MN: the monitors list passed to the system by the user function (main_NeuronGroup)

    This way it is possible to plot, save or do whatever you want with these objects after the end of the simulation!
    """
    pass
    figure()
    raster_plot(simulation_MN[1])
    title("Spikes Received by UDP")
    show(block=True) 
    # savefig('output.pdf')




if __name__=="__main__":

    my_simulation = BrianConnectUDP(main_NeuronGroup, NumOfNeuronsInput=135, post_simulation_function=post_simulation_function,
         UDP_IPI="127.0.0.1", UDP_PORTI = 22222, simclock_dt=1, inputclock_dt=2, TotalSimulationTime=10000, sim_repetitions=0)    
