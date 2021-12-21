#!/usr/bin/bash


ERROR_STRING="Solution found but controller failed during execution"
let counter=1

while :
do
    echo "-----------------------------------------------"
    if grep -q "$1" <<< "execute_cartesian_goal.py" && [ $counter -lt 2 ]; then
        echo "ENTER X,Y,Z,A,B,C VALUES"
	read -p "Give the value you want to move along the x-axis (in mm): " X
	read -p "Give the value you want to move along the y-axis (in mm): " Y
	read -p "Give the value you want to move along the z-axis (in mm): " Z
	read -p "Give the value you want to rotate around the x-axis (in degrees): " A
	read -p "Give the value you want to rotate around the y-axis (in degrees): " B
	read -p "Give the value you want to rotate around the z-axis (in degree): " C
	echo "RUNNING PYTHON SCRIPT FOR THE $counter TIMES"
    	python $1 $X $Y $Z $A $B $C > testfile2.txt
    elif grep -q "$1" <<< "execute_cartesian_goal.py" && [ $counter -gt 1 ]; then
	echo "RUNNING PYTHON SCRIPT FOR THE $counter TIMES"
        python $1 $X $Y $Z $A $B $C > testfile2.txt
    else
        echo "RUNNING PYTHON SCRIPT FOR THE $counter TIMES"
        python $1 > testfile2.txt
        sleep 0.01
    fi

    echo "STORING OUTPUT IN A FILE"
    file="/home/praktikant2/ws_moveit/src/project_praktikum_moveit_config/scripts/testfile2.txt"

    let flag=0

    while IFS= read -r line
    do
        if grep -q "$ERROR_STRING" <<< "$line"; then
	    echo "ERROR FOUND IN: $line"
	    let flag=1
	    break
        fi
        echo "LOOKING FOR ERRORS IN: $line "
        sleep 0.01

    done <"$file"

    echo "ERROR STATUS: $flag"

    if [ $flag -eq 0 ] || [ $counter -gt 4 ]
    then
	if [ $counter -gt 3 ]
	then
	    echo "MAXIMAL NUMBER OF ATTEMPTS ($counter) REACHED"
	else
	    echo "NO ERRORS FOUND"
	fi
	echo "STOP EXECUTING PYTHON SCRIPT"
        echo "DONE"
	break
    fi

    let counter+=1

done
