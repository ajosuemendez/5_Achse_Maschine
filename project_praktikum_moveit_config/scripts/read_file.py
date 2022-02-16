import re
import math
import numpy
lines = []
full_list = []
dump_test = []

# defining the atan function
myatan = lambda x,y: numpy.pi*(1.0-0.5*(1+numpy.sign(x))*(1-numpy.sign(y**2))\
         -0.25*(2+numpy.sign(x))*numpy.sign(y))\
         -numpy.sign(x*y)*numpy.arctan((numpy.abs(x)-numpy.abs(y))/(numpy.abs(x)+numpy.abs(y)))



with open("/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/scripts/halbkugel_dumper.txt") as f:
    lines= f.readlines()

for line in lines:
    if line.find("onLinear5D") > 0:
        x = [line]
        start = 0
        while True:
            index_e = x[0].find("e-", start)
            if index_e <0:
                break
            index_next_comma =  x[0].find(",",index_e)
            for i in range(index_e, 0,-1):
                if  x[0][i] == ",":
                    index_prev_comma = i
                    break
                else:
                    index_prev_comma = -1

            x[0] =  x[0].replace(x[0][index_prev_comma:index_next_comma], ", 0.000")
            start = index_e +1
    #
    #     print(x[0])

        pattern = re.compile(r'([.-]|)((\d*\.\d*)|(\d*))')
        matches = pattern.finditer(str(x[0]))
        tcp_coordinates = [match.group(0) for match in matches]
        tcp_coordinates = [elem for elem in tcp_coordinates if elem!=""]
        #print(tcp_coordinates)
        if len(tcp_coordinates) >9:
            del tcp_coordinates[0]
            del tcp_coordinates[0]
            dump_test.append(tcp_coordinates)
            tcp_coordinates = [round(float(elem), 2) for elem in tcp_coordinates]
            full_list.append(tcp_coordinates)

            # print(tcp_coordinates)


with open('/home/praktikant2/dumpTest/DebugTestDumper.txt', 'w') as f:
    for i in range(len(dump_test)):
        f.write(f"counter:{i}, {dump_test[i]}\n")



coordinates = [(round(math.degrees(myatan(elem[4], elem[3])), 3) , round(math.degrees(math.atan((math.sqrt( abs((elem[3])**2) + abs((elem[4])**2)))/elem[5])),3)) for elem in full_list]

for i in range(len(coordinates)):
    full_list[i].append(coordinates[i][0])
    full_list[i].append(coordinates[i][1])

    print(f"counter:{i}x: {full_list[i][0]} y: {full_list[i][1]} z: {full_list[i][2]} yaw: {full_list[i][-2]} pitch: {full_list[i][-1]} speed: {full_list[i][6]}" )
