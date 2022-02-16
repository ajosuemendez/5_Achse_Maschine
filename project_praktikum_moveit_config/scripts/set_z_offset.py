#!/usr/bin/env python

with open("/home/praktikant2/gcodes_examples/fix_speed_1000_kugel.txt") as f:
    lines= f.readlines()

a = []
offset = 200
for line in lines:
    z_index = line.find("Z")
    b_index = line.find("B")

    z_value = float(line[z_index+1:b_index]) + offset

    fixed_line = line.replace(f"{line[z_index+1:b_index]}", f"{z_value}")
    a.append(fixed_line)
    print(fixed_line)


with open('/home/praktikant2/gcodes_examples/reduced_30_step_kugel.txt', 'w') as f:
    f.writelines(a)
