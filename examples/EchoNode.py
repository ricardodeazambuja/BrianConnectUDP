'''
This is the Echo node
It sends back everything it receives.
'''

import argparse

import numpy

import time

from brian_multiprocess_udp import BrianConnectUDP


def echo_node(spikes_pipe_in, spikes_pipe_out):
    """
    This function substitutes the run_brian_simulation and is a echo node only.
    An important thing to pay attention here is that the format of the incomming spikes
    and the outgoing ones IS NOT THE SAME!
    """

    output = numpy.array(range(Number_of_Neurons))
    # Because the system receives lists with 1's or 0's from the spikes_pipe_in, but sends only the the
    # index where a spike occured using the spikes_pipe_out, it is necessary a minor translation before sending the echo.

    while True:

        # # DEBUG
        data = spikes_pipe_in.recv()
        # print data
        time.sleep(5E-3)
        spikes_pipe_out.send(output*data)
        # # DEBUG

        # spikes_pipe_out.send(output*spikes_pipe_in.recv())
        # Blocks until there its something to receive.



if __name__=="__main__":


    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Sets up and launch the ECHO node.")

    parser.add_argument("--neurons", help="The total number of neurons (INPUT/OUTPUT are the same size).", type=int, required=True)
    
    parser.add_argument("--output_port", help="Output port (integer).", type=int, required=True)
    parser.add_argument("--input_port", help="Input port (integer).", type=int, required=True)
    parser.add_argument("--output_ip", help="Output addresses (string).", type=str, required=True)

    parser.add_argument("--sim_clock", help="Simulation clock (float, milliseconds).", type=float, required=True)    
    parser.add_argument("--input_clock", help="Input clock (float, milliseconds).", type=float, required=True)    

    parser.add_argument("--brian_addr", help="A number from 0 to 255 used to identify the node.", type=int, default=255)
    parser.add_argument("--ttime", help="Total time of the simulation.", type=int, required=True)

    
    
    args=parser.parse_args()


    Number_of_Neurons = args.neurons


    my_simulation = BrianConnectUDP(main_NeuronGroup=None, 
        NumOfNeuronsOutput=Number_of_Neurons, NumOfNeuronsInput=Number_of_Neurons,
        simclock_dt=args.sim_clock, inputclock_dt=args.input_clock,
        input_addresses=[('localhost', args.input_port, Number_of_Neurons)], output_addresses=[(args.output_ip, args.output_port)], 
        TotalSimulationTime=args.ttime, brian_bypass=echo_node, brian_address=args.brian_addr)
