#!/usr/bin/env python
'''

This is a server side script that receives pitch and yaw and converts to spikes!

'''
import sys
sys.path.append("/Users/ricardodeazambuja/Dropbox/PhD_University/BrianTraining/LiquidStateMachine/")

from brian_multiprocess_udp import BrianConnectUDP

import numpy

import time

outputclock_dt = 200 #in milliseconds

Number_of_Neurons_Output = 4

def user_input_to_spikes(spikes_pipe_in, spikes_pipe_out):
    """
    This function substitutes the run_brian_simulation and converts the user input into spikes
    """

    print "User Input!" #DEBUG!

    while True:
        try:
            with open('/Users/Guest/Sites/temp.dat','r') as f:
                pitch = f.readline()
                yaw = f.readline()
        except IOError:
            print "IOError"
            pass        
        pitch = int(pitch)
        yaw = int(yaw)
        pitch_vector = []
        yaw_vector = []

        if pitch>240:
            pitch_vector = [0] # goes down
        elif pitch<225:
            pitch_vector = [1] # goes up
        

        if yaw>240:
            yaw_vector = [2] # goes right
        elif yaw<225:
            yaw_vector = [3] # goes left
        
        final_vector = pitch_vector+yaw_vector
        spikes_pipe_out.send(numpy.array(final_vector))

        print "Input Neuron Group"
        print "Pitch / Yaw: %s / %s" % (pitch,yaw)
        print "Spikes:%s" % final_vector
        
        time.sleep(outputclock_dt/1000.0)


if __name__=="__main__":

    my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsOutput=Number_of_Neurons_Output,
        output_addresses=[("192.168.1.111", 33333),("192.168.1.111", 31313)], TotalSimulationTime=600000, brian_bypass=user_input_to_spikes, brian_address=0)

    # my_simulation = BrianConnectUDP(main_NeuronGroup=None, NumOfNeuronsOutput=Number_of_Neurons_Output,
    #     output_addresses=[("127.0.0.1", 30303)], TotalSimulationTime=100000, brian_bypass=user_input_to_spikes, brian_address=0)    
