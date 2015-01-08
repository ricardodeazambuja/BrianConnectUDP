#!/usr/bin/env python

'''
This is the input node that receives spikes and generates a plot.
LINUX VERSION!
'''

import numpy
import socket
import sys
import argparse
import matplotlib.pyplot as plt
import time
from multiprocessing import Process, Pipe
from select import select

# import os, signal

# to read back the generated file
# import json
# with open('my_file') as f:
#     my_list = [json.loads(line) for line in f]

def converts_spikes_into_plot(spike_numpy_array, x, y, step):
    """
    spike_numpy_array => the numpy array received directly from my system (a numpy array with 1's and 0's) and the 
                         same length as NumOfNeuronsInput.

    returns a plot line object
    """

    for i,j in zip(spike_numpy_array,range(len(spike_numpy_array))):
        if i==1: # Is there a spike in the index j?
            x.append(step)
            y.append(j)

    return (x,y)


def saves_to_file(spikes_pipe_in, filename):
    """
    """
    start_time=time.time()*1000.0 # time in milliseconds
    while True:
        x=list()
        y=list()

        if select([spikes_pipe_in],[],[]):

            time_step=int(time.time()*1000.0 - start_time)


            x,y = converts_spikes_into_plot(spikes_pipe_in.recv(), x, y, time_step)

            with open(filename,"a") as f:
                f.write(str([x,y]))
                f.write('\n')


def draw_plots(spikes_pipe_in, cmd_pipe_in, number_of_neurons, save_file):

    
    print "Starting Online Spike Plot"

    fig = plt.figure()

    plt.title("Real time Spike Plot")


    step=0
    time_step=0
    start_time=time.time()*1000.0 # time in milliseconds
    x=list()
    y=list()

    # Until this point, the code is going to be executed only once each time the system runs.
    try:
        while True:

            if select([spikes_pipe_in],[],[]):

                if spikes_pipe_in.poll():
                    time_step=int(time.time()*1000.0 - start_time)

                    # plt.axis([start_time,time_step+1,-1,number_of_neurons])
                    plt.ylim(-1,number_of_neurons)
                    # Adjusts the axes according to the number of spikes published.

                    x,y = converts_spikes_into_plot(spikes_pipe_in.recv(), x, y, time_step)
                    # plt.scatter(x,y) 
                    plt.plot(x,y,'b.')

                    step+=1
                    if step > steps:
                        step=0
                        # start_time=time.time()*1000.0 # time in milliseconds
                        x=[]
                        y=[]
                        plt.pause(0.0001)
                        plt.clf()
                    # Here the figure is created / updated.

    except KeyboardInterrupt:
        if args.save != 'None': # Here is 'None' as a string because this is how the user can enter at command line.
            print 
            print "Saving figure! Wait..."
            fig.savefig(save_file)
            print "Saving figure! Wait...Done!"
            # sys.exit(0)


def read_UDP(pipe_out_draw, pipe_out_file, IPI, PORTI, number_of_neurons, clean):
    """
    This function simply creates a socket, reads all the UDP packets as they arrive and redirects to a multiprocessing.Pipe.

    IPI = "X.X.X.X" ordinary IP address from one of the network interfaces
    PORTI = 0 to 65535 (but you need to choose a free one)
    number_of_neurons = is the size of the spike train
    pipe_out = multiprocessing.Pipe used to send the information received through UDP to the draw_plots
    clear = when True cleans the receiving buffer before start
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

    if clean:
        clean_loop = 1
        while clean_loop:
            print "Cleaning receiving buffer...", "IP/PORT:", IPI, "/", PORTI
            try:
                data = sockI.recv(1, socket.MSG_DONTWAIT) # buffer size is 1 byte, NON blocking.
                print data
            except IOError: # The try and except are necessary because the recv raises a error when no data is received
                clean_loop = 0
        print "Cleaning receiving buffer...", "IP/PORT:", IPI, "/", PORTI, "...Done!"

    sockI.setblocking(1) # Tells the system that the socket recv() method will DO block until a packet is received            

    while True:

        # Receives the spike train from the pipe, converts according to the converts_spikes_into_plot function
        try:
            received_raw_data = sockI.recv(buffer_size) # This is a blocking command, therefore the while loop is not going
                                                        # to eat up all the processor time.

            numpy_data = numpy.fromstring(received_raw_data[8:], dtype=numpy.uint8)
            pipe_out_draw.send(numpy_data)
            if filename != None:
                pipe_out_file.send(numpy_data)
            
            # The first 8 bytes are the brian_address, so I don't need them here

        except IOError:  # Without the IOError even the keyboard "control+C" is caught here!
            print "UDP read error?"                        
            pass

        except ValueError:
            print "ValueError:", data # DEBUG!
            pass #the data is corrupted, a wrong package appeared at the port, etc...

        except KeyboardInterrupt:
            pass # Just to disable the msgs...




if __name__=="__main__":

    # Process the information received from the command line arguments.
    parser = argparse.ArgumentParser(description="Generate a \"real time\" plot of the spikes comming from the IP/PORT.")

    parser.add_argument("--IP", help="IP Address to receive the spikes.", type=str, default="192.168.1.100")
    parser.add_argument("--PORT", help="PORT to receive the spikes.", type=int, default=30303)
    parser.add_argument("--NON", help="Number of neurons the spike train has.", type=int, default=600)
    parser.add_argument("--steps", help="Number of steps to be plotted.", type=int, default=100)
    parser.add_argument("--file", help="Saves the data received to a file.", type=str, default=None)
    parser.add_argument("--save", help="Saves the figure after receive a Ctrl+C. If set as None, nothing is saved.", type=str, default="OnlineSpikePlot" + str(time.time()) + ".png")
    parser.add_argument("--clean", help="Cleans the receiving buffer before start.", action="store_true")
    
    args=parser.parse_args()

    steps = args.steps

    filename = args.file

    IPI = args.IP
    PORTI = args.PORT
    number_of_neurons = args.NON

    command_r, command_w = Pipe(duplex=False)

    UDP_draw_r, UDP_draw_w = Pipe(duplex=False)

    UDP_file_r, UDP_file_w = Pipe(duplex=False)

    generate_plots = Process(target=draw_plots, args=(UDP_draw_r, command_r, number_of_neurons, args.save))

    read_socket = Process(target=read_UDP, args=(UDP_draw_w, UDP_file_w, IPI, PORTI, number_of_neurons, args.clean))

    if filename != None:
        save_plots = Process(target=saves_to_file, args=(UDP_file_r, filename))

    generate_plots.daemon = True # Guarantees the process will die after the main python
    if filename != None:
        save_plots.daemon = True # Guarantees the process will die after the main python
    read_socket.daemon = True # Guarantees the process will die after the main python

    read_socket.start()
    generate_plots.start()
    
    if filename != None:
        save_plots.start()

    try:
        while True:
            print "Run!"
            if select([],[],[]):
                print "Never!"
                pass

    except KeyboardInterrupt:
        print
        if args.save != "None":
            time.sleep(5) # Gives time to save the image

        read_socket.terminate()
        generate_plots.terminate()
        if filename != None:        
            save_plots.terminate()
        time.sleep(.1)
        sys.exit("Exiting the online spike plot!")
        # os.kill(os.getppid(), signal.SIGQUIT) #or signal.SIGKILL
