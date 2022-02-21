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


def check_filter(line,*mode):
    name = mode[0][0]
    state = mode[0][1]
    if state:
        if line.find(name)>0:
            return name
        else:
            return "Not Found"
    else:
        return "Not Found"

with open("/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/scripts/onLinearTest.txt") as f:
    lines= f.readlines()

mode_onRapid5D = ("onRapid5D(",True)
mode_onLinear5D = ("onLinear5D(",True)
mode_all = True
if mode_all:
    mode_onLinear = ("onLinear(",True)
    mode_onRapid = ("onRapid(",True)
    mode_onCircular = ("onCircular(",True)
    mode_onRapid5D = ("onRapid5D(",True)
    mode_onLinear5D = ("onLinear5D(",True)
else:
    mode_onLinear = ("onLinear(",False)
    mode_onRapid = ("onRapid(",False)
    mode_onCircular = ("onCircular(",False)

modes = [mode_onLinear5D, mode_onRapid5D, mode_onLinear, mode_onRapid, mode_onCircular]

counter = 0
for line in lines:
    #if line.find("onLinear5D") > 0 or if line.find("onRapid5D">0):
    #if line.find("onRapid5D")>0:
    cw= True
    found_functions = []
    for i in modes:
        mode_name = check_filter(line, i)
        if mode_name != "Not Found":
            found_functions.append(mode_name)

    if found_functions:
        # print(found_functions)
        # print("new line")

        x = [line]
        start = 0
        flag_parenthesis = False
        while True:
            index_e = x[0].find("e-", start)
            if index_e <0:
                break
            index_next_comma =  x[0].find(",",index_e)
            #index_parenthesis = x[0].find("(",0)

            for i in range(index_e, 0,-1):
                if  x[0][i] == ",":
                    index_prev_comma = i
                    break
                elif x[0][i] == "(":
                    index_prev_comma = i
                    break
                else:
                    index_prev_comma = -1

            x[0] =  x[0].replace(x[0][index_prev_comma:index_next_comma], ", 0.000")
            start = index_e +1
    #
    #     print(x[0])
        if str(x[0]).find("false") > 0:
            cw = False

        pattern = re.compile(r'([.-]|)((\d*\.\d*)|(\d*))')
        matches = pattern.finditer(str(x[0]))
        tcp_coordinates = [match.group(0) for match in matches]
        tcp_coordinates = [elem for elem in tcp_coordinates if elem!=""]
        #print(tcp_coordinates)
        #print(found_functions)
        if len(tcp_coordinates) >1:
            if "onLinear(" in found_functions or "onRapid(" in found_functions or "onCircular(" in found_functions:
                del tcp_coordinates[0]
            elif "onRapid5D(" in found_functions or "onLinear5D(" in found_functions:
                del tcp_coordinates[0]
                del tcp_coordinates[0]

            dump_test.append(tcp_coordinates)
            tcp_coordinates = [round(float(elem), 2) for elem in tcp_coordinates]
            if "onRapid5D(" in found_functions:
                coordinate = [(round(math.degrees(myatan(tcp_coordinates[4], tcp_coordinates[3])), 3) , round(math.degrees(math.atan((math.sqrt( abs((tcp_coordinates[3])**2) + abs((tcp_coordinates[4])**2)))/tcp_coordinates[5])),3))]
                list_coordinate = [coordinate[0][0],coordinate[0][1]]
                full_list.append(tcp_coordinates + list_coordinate)
                print(f"Line Number:{counter} function(onRapid5D) x: {full_list[-1][0]} y: {full_list[-1][1]} z: {full_list[-1][2]} yaw: {full_list[-1][-2]} pitch: {full_list[-1][-1]}")
                counter +=1
            elif "onLinear5D(" in found_functions:
                coordinate = [(round(math.degrees(myatan(tcp_coordinates[4], tcp_coordinates[3])), 3) , round(math.degrees(math.atan((math.sqrt( abs((tcp_coordinates[3])**2) + abs((tcp_coordinates[4])**2)))/tcp_coordinates[5])),3))]
                list_coordinate = [coordinate[0][0],coordinate[0][1]]
                full_list.append(tcp_coordinates + list_coordinate)
                print(f"Line Number:{counter} function(onLinear5D) x: {full_list[-1][0]} y: {full_list[-1][1]} z: {full_list[-1][2]} yaw: {full_list[-1][-2]} pitch: {full_list[-1][-1]} speed: {full_list[-1][6]}" )
                counter +=1
            elif "onLinear(" in found_functions:
                full_list.append(tcp_coordinates)
                print(f"Line Number:{counter} function(onLinear) x: {full_list[-1][0]} y: {full_list[-1][1]} z: {full_list[-1][2]} speed: {full_list[-1][3]}" )
                counter +=1

            elif "onRapid(" in found_functions:
                full_list.append(tcp_coordinates)
                print(f"Line Number:{counter} function(onRapid) x: {full_list[-1][0]} y: {full_list[-1][1]} z: {full_list[-1][2]}" )
                counter +=1

            elif "onCircular(" in found_functions:
                full_list.append(tcp_coordinates)
                print(f"Line Number:{counter} function(onCirular) clockwise: {cw} cx: {full_list[-1][0]} cy: {full_list[-1][1]} cz: {full_list[-1][2]} end_x: {full_list[-1][3]} end_y: {full_list[-1][4]} end_z: {full_list[-1][5]}" )
                counter +=1


            #print(tcp_coordinates)
# print(full_list)

#uncomment these kines
# with open('/home/praktikant2/dumpTest/DebugTestDumper.txt', 'w') as f:
#     for i in range(len(dump_test)):
#         f.write(f"counter:{i}, {dump_test[i]}\n")
#
#
#
#print(found_functions)

# coordinates = [(round(math.degrees(myatan(elem[4], elem[3])), 3) , round(math.degrees(math.atan((math.sqrt( abs((elem[3])**2) + abs((elem[4])**2)))/elem[5])),3)) for elem in full_list]
#
# for i in range(len(coordinates)):
#     full_list[i].append(coordinates[i][0])
#     full_list[i].append(coordinates[i][1])
#
#     print(f"counter:{i}x: {full_list[i][0]} y: {full_list[i][1]} z: {full_list[i][2]} yaw: {full_list[i][-2]} pitch: {full_list[i][-1]} speed: {full_list[i][6]}" )
