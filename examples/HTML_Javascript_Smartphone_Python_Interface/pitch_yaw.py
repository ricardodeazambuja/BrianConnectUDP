#!/usr/bin/env python
import cgi

print "Content-type: text/html\n\n"


form = cgi.FieldStorage()

pitch = form.getfirst("pitch", "0") # get the form value associated with form
                               # name 'x'.  Use default "0" if there is none. 
yaw = form.getfirst("yaw", "0") # similarly for name 'y'

pitch_yaw = (int(pitch),int(yaw))

try:
    with open('temp.dat','w') as f:
        f.write(str(pitch)+'\n')
        f.write(str(yaw)+'\n')
except IOError:
    print "IOError"
    pass


print pitch + " " + yaw # Return to the webpage the values read

# # To read the file
# try:
#     with open('/Users/Guest/Sites/temp.dat','r') as f:
#         data1 = f.readline()
#         data2 = f.readline()
# except IOError:
#     print "IOError"
#     pass        
# data1 = int(data1)
# data2 = int(data2)    
