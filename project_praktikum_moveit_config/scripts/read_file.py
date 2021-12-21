import re
import math
import numpy
lines = []
full_list = []

# defining the atan function
myatan = lambda x,y: numpy.pi*(1.0-0.5*(1+numpy.sign(x))*(1-numpy.sign(y**2))\
         -0.25*(2+numpy.sign(x))*numpy.sign(y))\
         -numpy.sign(x*y)*numpy.arctan((numpy.abs(x)-numpy.abs(y))/(numpy.abs(x)+numpy.abs(y)))



with open("/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/scripts/5d_samples.txt") as f:
    lines= f.readlines()

for line in lines:

    pattern = re.compile(r'([.-]|)\d*\.\d*')
    matches = pattern.finditer(str(line))
    my_list = [round(float(match.group(0)), 3) for match in matches]
    full_list.append(my_list)

    #print(my_list)

coordinates = [(round(math.degrees(myatan(elem[4], elem[3])), 3) , round(math.degrees(math.atan((math.sqrt( abs((elem[3])**2) + abs((elem[4])**2)))/elem[5])),3)) for elem in full_list]

for i in range(len(coordinates)):
    full_list[i].append(coordinates[i][0])
    full_list[i].append(coordinates[i][1])

    print(f"x: {full_list[i][0]} y: {full_list[i][1]} z: {full_list[i][2]} yaw: {full_list[i][6]} pitch: {full_list[i][7]}" )
