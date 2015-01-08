'''
Example of a node that doesn't use Brian at all!

Instead of doing any simulation using Brian, this node only receives and sends back spike trains according to the 
formatting used by the BrianConnectUDP.

'''

import numpy
import select
import time

from brian_multiprocess_udp import BrianConnectUDP


my_inputclock_dt = 2

def redirect_spikes(spikes_pipe_in, spikes_pipe_out):
    """
    This function substitutes the run_brian_simulation and makes the system behaves like a node/router simply
    redirecting packets or receiving multiple packets and converting to a big one.
    """
    while True:
        if select.select([spikes_pipe_in],[],[]):
            t_init = time.time()

            spikes_pipe_out.send(numpy.array([index for index,value in enumerate(spikes_pipe_in.recv().tolist()) if value==1]))

            time.sleep((my_inputclock_dt/1000.0)-(time.time()-t_init)) # Keeps the same clock defined by the class call.
                                                                       # This could be useful to keep all the nodes with the
                                                                       # same density of spikes arriving (if they use similar 
                                                                       # values to the inputclock_dt)




if __name__=="__main__":

    my_simulation = BrianConnectUDP(None, NumOfNeuronsInput=100, NumOfNeuronsOutput=100,
        input_addresses=[("127.0.0.1", 14141, 40),("127.0.0.1", 16161, 60)], 
        output_addresses=[("127.0.0.1", 18181)], 
        inputclock_dt=my_inputclock_dt, TotalSimulationTime=10000, brian_bypass=redirect_spikes, brian_address=2)
