"""
Notes:
- Make possible to bypass the multiple input/output and make it only one input / one output to run faster.
- Check the possibility of using the AER format instead of my own or at least to develop a converter node or a option 
where it is possible to choose the format (mine or AER).
- Check the possibility of using multicast instead of sending multiple spikes.

Limitations (so far):
1) Maximum number of neurons
- Because I'm using Pipe, there is a buffer size limit that varies from system to system.

2) Another problem with the Pipe buffer
- If the simulation ran faster than the sockets can process the packets, the Pipe buffer will be full and block.

3) Maximum number of connections with other systems
- Each connection is using a new process, so the system can crash if the number of processes is too big.
- Also the number of sockets open could be a problem, but I don't think so.

4) Windows users:
- I have no idea if this code is going to work on Windows :(



BrianConnectUDP
Interface to connect multiple simulations using (or not) Brian through UDP packets.
BrianConnectUDP(main_NeuronGroup, 
                NumOfNeuronsInput=None, NumOfNeuronsOutput=None,
                post_simulation_function = None,
                output_addresses = None, input_addresses = None, 
                simclock_dt=1, inputclock_dt=None, TotalSimulationTime=1000, sim_repetitions=0, 
                brian_bypass=None, 
                brian_address=0)

main_NeuronGroup:
Function with the Brian simulation.
main_NeuronGroup(input_Neuron_Group, simulation_clock)

The main function with the NeuronGroup(s) and Synapse(s) must be named "main_NeuronGroup".
It will receive two objects: input_Neuron_Group and the simulation_clock. The input_Neuron_Group
will supply the input spikes to the network. The size of the spike train received equals NumOfNeuronsInput.
The size of the output spike train equals NumOfNeuronsOutput and must be the same size of the NeuronGroup who is
going to interface with the rest of the system to send spikes. The simulation_clock must be used within all the NeuronGroups
because Brian require all the interconnected NeuronGroups to use the same clock.

The function must return all the NeuronGroup objects and all the Synapse objects this way:
([list of all NeuronGroups],[list of all Synapses], [list of all Monitors])
and the FIRST (index 0) NeuronGroup of the list MUST be the one where the OUTPUT spikes will be taken by the simulation.

The user can also create functions using the @network_operation decorator. In this case, the functions must be returned 
together with the NeuronGroups.

If main_NeuronGroup=None, the system will by pass Brian using the function passed through "brian_bypass" instead.

post_simulation_function:
Function with the Brian code to be executed after each simulation run.
post_simulation_function(input_NG, simulation_NG, simulation_SYN, simulation_MN):
    input_NG: the neuron group that receives the input spikes
    simulation_NG: the neuron group list passed to the system by the user function (main_NeuronGroup)
    simulation_SYN: the synapses list passed to the system by the user function (main_NeuronGroup)
    simulation_MN: the monitors list passed to the system by the user function (main_NeuronGroup)

This way it is possible to plot, save or do whatever you want with these objects after the end of the simulation!
Useful to save weights, generate plots, etc.

NumOfNeuronsInput:
The number of neurons/spikes the simulation wants to receive by UDP. Defines the number of neurons at the
input layer (NeuronGroup) of the Brian simulation. If this is set to None (default), the node becomes a spike generator 
sending spikes through UDP packets, but without inputs from UDP packets.

NumOfNeuronsOutput:
The number of neurons/spikes the simulation wants to send by UDP. Defines the number of neurons at the
output layer (NeuronGroup) of the Brian simulation. If this is set to None (default), the node becomes a spike processor, 
receives spikes through UDP packet, but no spikes are sent through UDP packets.

output_addresses: 
It is a list of tuple(s) with all IP/PORT used to SEND the spikes 
[("IP", PORT_NUMBER),...]
IP: The IP address of the machine the user wants to send spikes to.
PORT: Port number to send spikes (check if the PORT is not being used by another process).

input_addresses: 
It is a list of tuple(s) with all IP/PORT and number of neurons used to RECEIVE the spikes 
[("IP", PORT_NUMBER, Number_of_Neurons),...]
IP: The IP address of the machine where this module is running. If everything is in the same machine, use the 127.0.0.1. 
In case the machine has multiple network interfaces, the user can define each one is going to be used by the IP address. 
Others machines who want to send spikes are going to use this address.
PORT: Port number to receive spikes (check if the PORT is not being used by another process).

simclock_dt: milliseconds!
Clock period utilized at the simulation.

inputclock_dt: milliseconds!
Period of time (ideal) the received spikes are checked and introduced in the network. Because this is implemented
using a network_operation with Brian, the period can vary according to the delay between realtime and the simulation time.

TotalSimulationTime:
How many milliseconds the simulation should run. When the brian_bypass is used, the simulation is killed after this time
has passed.

sim_repetitions:
The number of times the Brian net.run(...) command should be executed together with the post_simulation_function

brian_bypass:
Receives the function to be used when the main_NeuronGroup==None. This function must receive two multiprocessing.Pipe 
instances: the first (spikes_pipe_in in the example below) with the spikes received by UDP and the second (spikes_pipe_out)
with the spikes to be sent through UDP.
The received spikes (spikes_pipe_in) is a numpy.array with a number one (1) at the position the spike occured and zero (0) 
otherwise. The size of the received spike train is the same size of the NumOfNeuronsInput.
On the other hand, the spikes sent through UDP is a numpy.array with the indexes where a spike occured. If no spikes occur
nothing is sent by UDP. If only the neuron number 'x' generated a spike, then a numpy.array with the number 'x' is sent.
redirect_spikes(spikes_pipe_in, spikes_pipe_out)

brian_address:
An integer number from 0 to 255 representing the node. It is for future use only.


To be able to use this module it is necessary to create a function: 
main_NeuronGroup(input_Neuron_Group, simulation_clock)
Where input_Neuron_Group is going to be a NeuronGroup used to get the spikes received by UDP and the simulation_clock
is the clock to be utilized within the NeuronGroups. 

The function main_NeuronGroup must return a tuple with three lists. The first list must have as the index zero the 
NeuronGroup that is going to be used to generate the spikes to be sent by UDP (if an address was supplied). All others 
groups and network_operation functions must be inserted in this list too. The second list must have all the Synapses 
and the third list must have all the monitors.


# # # # # # # # # # # # ###########################################################
How the spikes are encoded (or how to interface to this system from anything else):

In order to be able to talk to this system, it is necessary to send / receive an UDP packet where the payload
is a numpy.array (using dtype=uint8) converted to string using the numpy.tostring method to/from the right IP/PORT.

Actually, the original list (before being converted) has this format:
original_list[0:8]=>brian address
- It is a 8-bit binary representation of a integer from 0-255. Ex.: 3 => [0,0,0,0,0,0,1,1]
original_list[8:]->spikes
- The lenght of this part is the same as the number of neurons sending or receiving spikes. The system recognize a
spike when a position in the list has an number one (1). Ex.: [0,1,0,0,0,0,0,1,0,0] this means that there are a total
of ten (10) neurons and the neurons at positions 2 and 8 spiked (remember, the first position is zero).

So, aggregating both examples:
original_list => [0,0,0,0,0,0,1,1] + [0,1,0,0,0,0,0,1,0,0] => [0,0,0,0,0,0,1,1,0,1,0,0,0,0,0,1,0,0]
The UDP payload, in this example, will be a string (formed by bytes) containing:
\x00\x00\x00\x00\x00\x00\x01\x01\x00\x01\x00\x00\x00\x00\x00\x01\x00\x00
and if the user tries to print it out, nothing is going to appear because there is no character to represent this 
hexadecimal numbers.
\x00\x00\x00\x00\x00\x00\x01\x01 => the first 8 bytes representing the brian address
\x00\x01\x00\x00\x00\x00\x00\x01\x00\x00 =>the last 10 bytes representing the spikes generated by a layer / neuron group

In Python:
Generate the correct array format using original_list as example:
To send the spikes from original_list: numpy.array(original_list, dtype=numpy.uint8).tostring()
To receive the spikes form the string msg received by UDP: numpy.fromstring(msg, dtype=numpy.uint8)
"""

import brian_no_units # Speeds up Brian by ignoring the units
from brian import *

from multiprocessing import Process, Pipe
import numpy
import socket
import time
import sys

import select
import signal

import os

class BrianConnectUDP(object):

    def __init__(self, main_NeuronGroup, NumOfNeuronsInput=None, NumOfNeuronsOutput=None,
                       post_simulation_function = None,
                       output_addresses = None, 
                       input_addresses = None, 
                       simclock_dt=1, inputclock_dt=None, TotalSimulationTime=1000, sim_repetitions=0, brian_bypass=None,
                       brian_address=0):

            

        # Variables to be set up to the specifics necessities of each simulation
        self.brian_address = brian_address
        self.brian_address_unpacked = numpy.unpackbits(numpy.array([brian_address], dtype=numpy.uint8)) 
                                            # brian_address must be a integer from 0 to 255
                                            # then this command translates do binary and generates a numpy.array

        self.NumOfNeuronsInput = NumOfNeuronsInput # Number of neurons at the INPUT (spikes comming FROM outside)
        self.NumOfNeuronsOutput = NumOfNeuronsOutput # Number of neurons at the OUTPUT (spikes going TO outside)
        self.simclock_dt = simclock_dt # (in mS) clock of the NeuronGroups (if they are connected they must have the same clock!)
        self.inputclock_dt = inputclock_dt # (in mS) clock used to update the weights inside the Brian simulation (@network_operation)
        self.TotalSimulationTime = TotalSimulationTime #(in mS)
        self.main_NeuronGroup = main_NeuronGroup #function with the main Brian NeuronGroup

        self.brian_bypass = brian_bypass # function used when Brian is bypassed

        self.post_simulation_function = post_simulation_function #function with the Brian code to be executed after the simulation

        self.simulation_repetitions = sim_repetitions

        self.input_addresses = input_addresses # [("IP", PORT_NUMBER, Number_of_Neurons),...]
        
        self.output_addresses = output_addresses # [("IP", PORT_NUMBER),...]

        self.old_time_sync = 0 # Used inside the @network_operation function who slows down the simulation if it is running
                               # faster than realtime. What happens here is when there is no input spikes Brian speed up and
                               # so it is necessary to hold it down to avoid finishing the simulation before the spikes start
                               # comming to the system.

        self.multiprocessing_start() # Calls the main function to set up everything and start all the processes.


    def run_brian_simulation(self, spikes_pipe_in, spikes_pipe_out):
        """
        Calls the main_NeuronGroup method and effectively start the Brian simulation.
        - Nothing should be altered here. All the set up must be done inside the main_NeuronGroup function!
        """
        print "Starting run_simulation!" #DEBUG!

        simclock = Clock(dt=self.simclock_dt*ms) # clock of the NeuronGroups (if they are connected they must have the same clock!)

        if self.simclock_dt==self.inputclock_dt or self.inputclock_dt==None:
            inputclock = simclock # clock to update the inputs
        else:
            inputclock = Clock(dt=self.inputclock_dt*ms) # clock to update the inputs
        
        #############################################################################################
        # Slows down the simulation when it is running too fast (e.g.: no input spikes)
        ############################################################################################
        realtimeclock_dt = 10*self.simclock_dt # this value is 10 times bigger trying not to interfere (too much!) when 
                                               # the simulation is slower than realtime.
        realtimeclock = Clock(dt=realtimeclock_dt*ms)
        ############################################################################################

        if self.NumOfNeuronsInput:
            # This neuron group is only a interface to the input spikes
            # Why is it necessary? This way it is possible to easily distribute spikes to the synapses and it will 
            # always controls the same number of weights as the number of inputs.
            Ndummy=NeuronGroup(self.NumOfNeuronsInput, model='v:1', reset=0, threshold=0.5, clock=simclock)
            # the threshold is configured to 0.5 because the spikes will be generated by a voltage equals to 1 (bit)
            Ndummy.v=0 # no spikes at the beginning of the simulation

        else:
            Ndummy = [] # this means that there are no input spikes, therefore no input NeuronGroup!

        # 
        #
        # HERE COMES THE REAL NEURON NETWORK TO BE SIMULATED!
        # The real network is going to receive the neuron group Ndummy as input and the simulation clock (all connected 
        # groups must have the same clock object!) and return: 
        #     tuple([list of the neurons groups used internally],[list of the synapses used internally], [list of the monitors])
        # The FIRST neuron group of the list MUST be the one who is going to expose the OUTPUT spikes if there is at least
        # one IP/PORT configured as output!

        print "Calling user defined NeuronGroup(s) / Synapse(s)!" #DEBUG!
        neurongroups_sim, synapses_sim, monitors_sim = self.main_NeuronGroup(Ndummy, simclock)

        # END OF THE REAL NEURON NETWORK
        #
        #


        #
        # FUNCTIONS TO RECEIVE AND SEND SPIKES THROUGH UDP
        # Because this function is called a lot of times (inputclock), it is extremelly important to make it as light weight as possible.
        @network_operation(clock=inputclock)
        def generates_input_spikes():
            if spikes_pipe_in.poll(): # Verify if there is spikes in the pipe (multiprocessing.Pipe)
                                      # This "if" is important because the method recv() is a blocking one.
                Ndummy.v = spikes_pipe_in.recv() # generates spikes by setting the voltage above the threshold (>0.5)
                                                 # the received spikes come in a numpy.array with 1's and 0's


        @network_operation(clock=realtimeclock)
        def delay_to_realtime(): # keeps the simulation running near to real time if it is too fast
            if self.old_time_sync:
                delta_t=time.time()-self.old_time_sync
                if delta_t<realtimeclock_dt/1000.0: # "/1000.0" is to convert from seconds to miliseconds
                    # time.sleep((realtimeclock_dt/1000.0)-delta_t) # Checks if the simulation is faster and then
                    #                                               # calls sleep to make it slower.
                    select.select([spikes_pipe_in],[],[], (realtimeclock_dt/1000.0)-delta_t)
                                                                    # Checks if the simulation is faster and then
                                                                    # calls sleep to make it slower.
                                                                    # Using select instead of sleep the system keeps 
                                                                    # alert to input spikes and not only falls sleep :D

            self.old_time_sync = time.time()


        # Every time spikes occur at the SpikeMonitor related to the output neuron group, this function is called
        # So it is extremelly important to make it as light weight as possible.
        def setup_send_UDP(spikes):
            if len(spikes): #this looks funny, why the function is called without spikes if it is a spike monitor????
                spikes_pipe_out.send(spikes)
                # If the array with the spikes indexes is bigger than the maximum Pipe buffer, or the information
                # was not processed by the next stage, this command is going to block until the buffer is free!

        #
        # END OF FUNCTIONS TO RECEIVE AND SEND SPIKES THROUGH UDP
        #                


        if not self.NumOfNeuronsInput:
            generates_input_spikes = [] # deactivate the @network_operation that reads spikes into the simulation 
                                        # if the number of input neurons is ZERO.


        # This is the monitor responsable to call the function to send the generated spikes through UDP
        MExt_sensor=[]
        if self.NumOfNeuronsOutput:
            MExt_sensor=[SpikeMonitor(neurongroups_sim[0], record=False, function=setup_send_UDP)]
            # The monitor is only defined if there is output neurons!
        
        tinit=time.time() # saves start time


        print "Creating the Network!" #DEBUG!
        net = Network(neurongroups_sim + [Ndummy, generates_input_spikes, delay_to_realtime] + synapses_sim + monitors_sim + MExt_sensor)
        # Hint: when lists are summed they are concatenated, therefore an empty list does nothing!

        try: # The try is here because you only want to save something after running the simulation...
            for i in range(self.simulation_repetitions+1):
                print "Running the simulation!" #DEBUG!

                while spikes_pipe_in.poll():
                    trash = spikes_pipe_in.recv() # cleans the pipe before starting the simulation, otherwise when the simulation starts
                                                  # a lot of spikes can be generated at once.

                net.run(self.TotalSimulationTime*ms)

                print "Simulation time:",(time.time()-tinit)
                

                if self.post_simulation_function:
                    self.post_simulation_function(input_NG=Ndummy, simulation_NG=neurongroups_sim, simulation_SYN=synapses_sim, simulation_MN=monitors_sim)
        
        except KeyboardInterrupt: # Gives the user the option to process some information when the program receives a "Control+C". Useful to finish a too long simulation without losing the results.
            print
            print "Processing the KeyboardInterrupt..."
            if self.post_simulation_function:
                self.post_simulation_function(input_NG=Ndummy, simulation_NG=neurongroups_sim, simulation_SYN=synapses_sim, simulation_MN=monitors_sim)
            print "Processing the KeyboardInterrupt...Done!"


    def receive_UDP(self, IPI, PORTI, number_of_neurons, pipe_out):
        """
        This function simply creates a socket, reads all the UDP packets as they arrive and redirects to a multiprocessing.Pipe.

        IPI = "X.X.X.X" ordinary IP address from one of the network interfaces
        PORTI = 0 to 65535 (but you need to choose a free one)
        number_of_neurons = is the size of the spike train
        pipe_out = multiprocessing.Pipe used to send the information received through UDP to the format_received_spks
        """

        buffer_size = 8 + number_of_neurons # Each element of the numpy.array with the uint8 occupies 1 byte.
                                            # So, the brian_address has 8 elements, therefore 8 bytes. 
                                            # number_of_neurons: because each neuron occupies 1 byte (numpy.uint8)

        sockI = socket.socket(socket.AF_INET,    # IP
                              socket.SOCK_DGRAM) # UDP

        sockI.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Tells the OS that if someone else is using the PORT, it
                                                                    # can use the same PORT without any error/warning msg.
                                                                    # Actually this is useful because if you restart the simulation
                                                                    # the OS is not going to release the socket so fast and an error
                                                                    # could occur.

        sockI.bind((IPI, PORTI)) # Bind the socket to the IPI/PORTI

        # Add here a test to check if the user asked to clean the buffer before start.
        clean_loop = 1
        while clean_loop:
            print self.brian_address, "- Cleaning receiving buffer...", "IP/PORT:", IPI, "/", PORTI
            try:
                data = sockI.recv(1, socket.MSG_DONTWAIT) # buffer size is 1 byte, NON blocking.
                print data
            except IOError: # The try and except are necessary because the recv raises a error when no data is received
                clean_loop = 0
        print self.brian_address, "- Cleaning receiving buffer...", "IP/PORT:", IPI, "/", PORTI, "...Done!"

        sockI.setblocking(1) # Tells the system that the socket recv() method will DO block until a packet is received            

        # Until this point, the code is going to be executed only once each time the system runs.

        while True:
            try:
                pipe_out.send(sockI.recv(buffer_size)) # This is a blocking command, therefore the while loop is not going
                                                       # to eat up all the processor time.

            except IOError:  # Without the IOError even the keyboard "control+C" is caught here!
                print "UDP read error?"                        
                pass

            except ValueError:
                print "ValueError:", data # DEBUG!
                pass #the data is corrupted, a wrong package appeared at the port, etc...

            except KeyboardInterrupt:
                print "User keyboard interruption...finishing!"
                break # kills the while


    def format_received_spks_multiple(self, udp_data, process_out, addresses_input, spike_flag):
        """
        This function processes the information received by UDP using the receive_UDP creating a proper spike train to be
        fed to Brian.

        udp_data: list with all the pipes connections from the input (read_UDP processes)
        process_out: pipe to send the final spikes to Brian
        addresses_input: list with the information about the inputs [("IP", PORT_NUMBER, Number_of_Neurons),...]
        spike_flag: flag Brian is going to use to signalize the spikes were processed. This is important because makes
        possible to drop packets. Actually it is the consumer point of the process_out's pipe.


        """
        individual_num_of_neurons = [i[2] for i in addresses_input] # just filters from the address_input list to make it easier to read.
        
        formatted_train = numpy.array([0]*sum(individual_num_of_neurons)) # generates the empty numpy.array to receive the spikes in the right position/offset.
        
        offset_num_of_neuron = [(sum(individual_num_of_neurons[0:i]),individual_num_of_neurons[i]) for i in range(len(individual_num_of_neurons))]
                                                            # this one is a list of tuples with the initial offset and the total number of neurons
                                                            # it is used to position the spike received within the formatted_train numpy.array.

        pipe_and_offset = zip(udp_data, offset_num_of_neuron) # I'm doing this here to save processing cycles inside the while loop.

        def handler_clean_spike_train(signum, frame):
            """ 
            Function used to react according to the signal.alarm, send (or not) the spikes and clean the spike train.

            """
            if not spike_flag.poll() and any(formatted_train): # checks if Brian already emptied the consumer side of the pipe
                                                               # and if there is at least one spike to be processed.
                process_out.send(formatted_train)

            formatted_train.fill(0) #.fill(0) because is faster than assignment!

        # Sets the signal handler, so every time alarm reaches the final time, calls handler_clean_spike_train.
        signal.signal(signal.SIGALRM, handler_clean_spike_train)

        while True: 
            try:
                #
                # I need to check the best value for this alarm!!!!
                #
                signal.setitimer(signal.ITIMER_REAL, self.inputclock_dt/1000.0)
                                    # Sets the alarm to signal according to the Brian clock used to read the inputs.
                                    # If the alarm uns off, the spikes are trashed.
                                    # This is the only way to interrupt the select() call.
                                    

                if select.select(udp_data,[],[]): # It blocks until at least one of the pipes in the udp_data list gets data in!
                                           # select() should work in most of the operational systems, but if only linux is used
                                           # this function could be swapped by the poll and then waste less time with this test.
                    
                    signal.setitimer(signal.ITIMER_REAL, 0) # disables the alarm used to clean the formatted_train

                    for pipe_udp, offset in pipe_and_offset:

                        if pipe_udp.poll(): # Veryfies if there is any spiking waiting in this pipe.
                                            # This test is important because recv() is a blocking command.
                            received_raw_data = pipe_udp.recv() #reads from the pipe the data receive by UDP
                            received_spikes = numpy.fromstring(received_raw_data[8:], dtype=numpy.uint8)
                                                    # The dtype=numpy.uint8 is very important because the
                                                    # convertion to/from string depends on it! 
                                                    # Takes out the first 8 bytes related to the brian_address.
                            try:
                                formatted_train[offset[0]:offset[0]+offset[1]]=received_spikes
                            except ValueError:
                                print "Received packet Error!"
                                # in case the system receives a wrong packet in this address.

            except select.error: # Catches the exception raised by the alarm interruption inside the select command.
                pass             # In other words: makes possible to quit the select loop!

            except KeyboardInterrupt:
                print "User keyboard interruption...finishing!"
                break # Kills the while...

    def send_UDP(self, IP, PORT, pipe_out):
        """
        This is a very light weight function to send the final msg (local_brian_address + spikes_train) using UDP.
        The spikes_train is going to be a numpy.array converted to string using the numpy.tostring method and
        the local_brian_address the same format, but with the original numpy.array containing the address (8bits).
        Each IP/PORT is going to have a process running exclusively with this function and because this is an IO
        function it is not a big deal to have more processes than real processors.
        """
        sockO = (socket.socket(socket.AF_INET,     # IP
                               socket.SOCK_DGRAM)) # UDP
        while True:
            try:
                sockO.sendto(pipe_out.recv(), (IP, PORT)) # here it is being supposed the pipe has the final string
                                                          # the method .recv() is a blocking command

            except IOError: # Without the IOError even the keyboard "control+C" is caught here!
                print self.brian_address, "-send_UDP IO error?"
            
            except KeyboardInterrupt:
                print "User keyboard interruption...finishing!"
                break # Kills the while...                

    def send_UDP_output_pipes(self, local_brian_address, addresses_output, pipe_out, send_UDP_pipes):
        """
        This function receives a Pipe (pipe_out) where the spikes are going to arrive from the Brian simulation,
        sets up the output spike train (because Brian sends only the indexes of the neurons who spiked),
        converts to the numpy.tostring() format, adds the brian_address and then redirects to the processes who send
        the UDP packet to each address/machine.

        In order to make it a little bit faster, the local_brian_address could be converted to string before sending
        it to this function.
        """
        UDP_spike_array = numpy.array([0]*self.NumOfNeuronsOutput, dtype=numpy.uint8) 
                                                                # This command is outside the setup_send_UDP because 
                                                                # it slows down the function.
                                                                # The dtype=numpy.uint8 is very important because the
                                                                # convertion to/from string depends on it! 

        random_indexes = range(len(addresses_output)) # Creates the index to be shuffled next.
                                                           # in my experiments, the shuffle runs 10x faster with a list
                                                           # instead of using a numpy.array ?!?!?!?

        while True:
            try:
                numpy.random.shuffle(random_indexes) # Randomize (shuffle) the indexes, so on average they all receive
                                                     # packets with the same delay.

                UDP_spike_array.fill(0) # in my experiments, this is 10x faster than using [0]*NumOfNeuronsOutput for an array with 1k uint8s
                UDP_spike_array[pipe_out.recv()]=1 # now the place where a spike occured will have a number one     
                                                   # in my experiments, if the pipe_out.recv() is a list instead of numpy.array, 
                                                   # this command is 10x slower!
                                                   # .recv() is a blocking command, so no CPU power when no spikes are produced.

                spikes_to_send = local_brian_address.tostring()+UDP_spike_array.tostring() 
                                                                   # After this point, the output msg is ready to be sent

                for i in random_indexes:
                    send_UDP_pipes[i].send(spikes_to_send)        # Any time the Brian simulator send spikes through the
                                                                  # spikes_pipe_in, the spikes are going to be received and sent to the
                                                                  # processes dealing with the sockets/UDP
                                                                  # Because .recv() is a blocking command, there is no crazy
                                                                  # consumption of processor power here.
            except KeyboardInterrupt:
                print "User keyboard interruption...finishing!"
                break # Kills the while...                                                                  

    def multiprocessing_start(self):

        #
        # I think, maybe, I need something like a try: to catch the control+C and close all the processes.

        print self.brian_address, "- Starting BrianConnectUDP..."
    
        #self.brian_address 
        # Always use this when printing messages because make it easier
        # to debug when more than one instance is running in the same terminal.
        # I know that in a multiprocessing sharing stdout makes a mess, but it is better than nothing...
    
        print self.brian_address, "- Number of Input Neurons: ", self.NumOfNeuronsInput
        if self.NumOfNeuronsInput:
            i = 1
            for IPI, PORTI, NuOfNe in self.input_addresses:
                print self.brian_address, "- ", i, ") UDP target IPI/PORTI/NoOfNeurons: ", IPI, "/", PORTI, "/", NuOfNe
                i = i+1
    
        print self.brian_address, "- Number of Output Neurons: ", self.NumOfNeuronsOutput
        if self.NumOfNeuronsOutput:
            i = 1
            for IPO, PORTO in self.output_addresses:
                print self.brian_address, "- ", i, ") UDP target IPO/PORTO: ", IPO, "/", PORTO
                i = i+1



        # Pipe used to receive spikes from the UDP port
        # GetSpikes_w => is written by the read_UDP process
        # GetSpikes_r => is read by Brian
        GetSpikes_r, GetSpikes_w = Pipe(duplex=False)

        # Pipe used to send spikes to the UDP port
        # SendSpikes_w => is written by Brian
        # SendSpikes_r => is read by the send_UDP_output_pipes process
        SendSpikes_r, SendSpikes_w = Pipe(duplex=False)

        #
        # Setup to RECEIVE multiple (or not) spike trains by UDP        

        if self.NumOfNeuronsInput:
            print self.brian_address, "- Configuring receive_UDP..."

            read_UDP_processes_lst = [] # stores all the processes created (consumers)
            read_UDP_pipes = [] # stores the entrances of the pipes utilized with the processes to send the UDP packets (producers)
            for IP, PORT, NuOfNe in self.input_addresses:
                consumer, producer = Pipe(duplex=False)
                read_UDP_pipes.append(consumer)
                read_UDP_processes_lst.append(Process(target=self.receive_UDP, args=(IP, PORT, NuOfNe, producer)))

            procs_format_received_spks_multiple = Process(target=self.format_received_spks_multiple, args=(read_UDP_pipes, GetSpikes_w, self.input_addresses, GetSpikes_r))

            print self.brian_address, "- Configuring receive_UDP...Done!"


        #
        # Setup to SEND multiple (or not) spike trains by UDP
        if self.NumOfNeuronsOutput:
            print self.brian_address, "- Configuring send_UDP..."

            send_UDP_processes_lst = [] # stores all the processes created (consumers)
            send_UDP_pipes = [] # stores the entrances of the pipes utilized with the processes to send the UDP packets (producers)
            for IP, PORT in self.output_addresses:
                consumer, producer = Pipe(duplex=False)
                send_UDP_pipes.append(producer)
                send_UDP_processes_lst.append(Process(target=self.send_UDP, args=(IP, PORT, consumer)))
            
            procs_send_multiple_UDP=Process(target=self.send_UDP_output_pipes, args=(self.brian_address_unpacked, self.output_addresses, SendSpikes_r, send_UDP_pipes))

            # Instead of using individual processes, it is possible to use a pool of workers with the Pool class.
            # Using the pool of workers, one can limit the maximum number of processors to be used, but because it is
            # an IO thing I don't believe the processes are going to hijack all the CPU's power.

            print self.brian_address, "- Configuring send_UDP...Done!"

        if self.main_NeuronGroup != None:
            print self.brian_address, "- Configuring Brian..."
            briansimulation = Process(target=self.run_brian_simulation, args=(GetSpikes_r, SendSpikes_w))
            print self.brian_address, "- Configuring Brian...Done!"
        else:
            print self.brian_address, "- Configuring Bypass..."
            briansimulation = Process(target=self.brian_bypass, args=(GetSpikes_r, SendSpikes_w))
            print self.brian_address, "- Configuring Bypass...Done!"                    

        
        # Start the processes

        if self.NumOfNeuronsInput:       
            print self.brian_address, "- Starting read_UDP..."
            for procs_receive_udp in read_UDP_processes_lst:
                procs_receive_udp.daemon = True # Guarantees the process will die after the main python
                procs_receive_udp.start()

            procs_format_received_spks_multiple.daemon = True # Guarantees the process will die after the main python
            procs_format_received_spks_multiple.start()

            print self.brian_address, "- Starting read_UDP...Done!"

        if self.NumOfNeuronsOutput: 
            print self.brian_address, "- Starting send_UDP..."
            for procs_send_udp in send_UDP_processes_lst:
                procs_send_udp.daemon = True # Guarantees the process will die after the main python
                procs_send_udp.start()
            print self.brian_address, "- Starting send_UDP...Done!"
            
            print self.brian_address, "- Starting send_UDP_output_pipes..."
            procs_send_multiple_UDP.daemon = True # Guarantees the process will die after the main python
            procs_send_multiple_UDP.start()
            print self.brian_address, "- Starting send_UDP_output_pipes...Done!"


        print self.brian_address, "- Starting Brian simulation..."
        briansimulation.daemon = True # Guarantees the process will die after the main python
        briansimulation.start()
        print self.brian_address, "- Starting Brian simulation...Done!"

        def kills_processes(signum, frame):

            if signum!=None:
                raise KeyboardInterrupt # That way my Brian code will know that it is supposed to run the 

            if self.post_simulation_function:
                while briansimulation.is_alive(): # Gives time to the post_simulation_function to be finished
                    time.sleep(0.1)

            print str(self.brian_address) + " - Killing all the processes..."

            if self.NumOfNeuronsInput:
                for procs_receive_udp in read_UDP_processes_lst:
                    procs_receive_udp.terminate()
                procs_format_received_spks_multiple.terminate()
            if self.NumOfNeuronsOutput:
                for procs_send_udp in send_UDP_processes_lst:
                    procs_send_udp.terminate()
                procs_send_multiple_UDP.terminate()
            briansimulation.terminate()
            print str(self.brian_address) + " - Killing all the processes...Done!"
            sys.exit(str(self.brian_address) + " - Time is up... forced exit!")
            # os.kill(os.getppid(), signal.SIGQUIT) #or signal.SIGKILL 
            # Sometimes, under Ubuntu 12.04, the processes don't die. I need to solve this problem...

        if self.main_NeuronGroup == None:
            # Sets the signal handler, so every time alarm reaches the final time, calls kills_processes
            signal.signal(signal.SIGALRM, kills_processes)

            signal.setitimer(signal.ITIMER_REAL, (self.TotalSimulationTime)/1000) 
                                                # Kills the parent if the brian_pass function never exits.
                                                # This way all the daemons are killed too and the system is clean again :)
        try:
            briansimulation.join() #this means my program will wait until the process (simulation) is over
            # Because I set all the other processes with .daemon=True, after this part they will be all automatically killed!

        except KeyboardInterrupt:
            kills_processes(None, None)