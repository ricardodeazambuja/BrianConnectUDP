'''
This is the Sender Node.

It sends the start spike train and then simply receives and sends spikes using Brian NeuronGroups. At the end of 
the simulation the data collected is plotted.

'''

from brian import *

import argparse

from brian_multiprocess_udp import BrianConnectUDP


def main_NeuronGroup(input_Neuron_Group, simulation_clock):
    print "main_NeuronGroup!" #DEBUG!

    simclock = simulation_clock

    Nr=NeuronGroup(Number_of_Neurons, model='v:1', reset=0, threshold=0.5, clock=simclock)
    Nr.v=1

    # SYNAPSES BETWEEN REAL NEURON NETWORK AND THE INPUT
    Syn_iNG_Nr=Synapses(input_Neuron_Group, Nr, model='w:1', pre='v+=w', clock=simclock)

    Syn_iNG_Nr[:,:]='i==j' # It is a one-to-one connection configuration

    Syn_iNG_Nr.w=1 # So it spikes every time it receives an spike (threshold is lower than one).

    MExt=SpikeMonitor(Nr) # Spikes sent by UDP

    Mdummy=SpikeMonitor(input_Neuron_Group) # Spikes received by UDP

    return ([Nr],[Syn_iNG_Nr],[MExt, Mdummy])


def post_simulation_function(input_NG, simulation_NG, simulation_SYN, simulation_MN):
    """
    input_NG: the neuron group that receives the input spikes
    simulation_NG: the neuron groups list passed to the system by the user function (main_NeuronGroup)
    simulation_SYN: the synapses list passed to the system by the user function (main_NeuronGroup)
    simulation_MN: the monitors list passed to the system by the user function (main_NeuronGroup)

    This way it is possible to plot, save or do whatever you want with these objects after the end of the simulation!
    """
    figure()
    ylim(-1,Number_of_Neurons)
    raster_plot(simulation_MN[0])
    raster_plot(simulation_MN[1])
    # plot(simulation_MN[0][0],'b.')
    # plot(simulation_MN[1][0],'g.')
    title("Spikes \"start to be\" Sent(blue)/Received(green) by UDP")

    show(block=True) 
    # savefig('output.pdf')




if __name__=="__main__":


    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Sets up and launch the SENDER node.")

    parser.add_argument("--neurons", help="The total number of neurons (INPUT/OUTPUT are the same size).", type=int, required=True)
    
    parser.add_argument("--output_port", help="Output port (integer).", type=int, required=True)
    parser.add_argument("--input_port", help="Input port (integer).", type=int, required=True)
    parser.add_argument("--output_ip", help="Output addresses(string).", type=str, required=True)

    parser.add_argument("--sim_clock", help="Simulation clock (float, milliseconds).", type=float, required=True)    
    parser.add_argument("--input_clock", help="Input clock (float, milliseconds).", type=float, required=True)        

    parser.add_argument("--brian_addr", help="A number from 0 to 255 used to identify the node.", type=int, default=0)
    parser.add_argument("--ttime", help="Total time of the simulation.", type=int, required=True)

    
    
    args=parser.parse_args()


    Number_of_Neurons = args.neurons


    my_simulation = BrianConnectUDP(main_NeuronGroup=main_NeuronGroup, post_simulation_function=post_simulation_function,
        NumOfNeuronsOutput=Number_of_Neurons, NumOfNeuronsInput=Number_of_Neurons,
        simclock_dt=args.sim_clock, inputclock_dt=args.input_clock,
        input_addresses=[('localhost', args.input_port, Number_of_Neurons)], output_addresses=[(args.output_ip, args.output_port)], 
        TotalSimulationTime=args.ttime, brian_address=args.brian_addr)
