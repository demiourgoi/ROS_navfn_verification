#! /bin/bash

# Check all the tests in the tests/ directory stopping as soon as it finds
# one discrepancy

for test in $(find tests/ -name 'test_*.txt')
do
    echo "Testing $test"
    DIRNAME=$(dirname $test)
    ROS_OUT="$DIRNAME/ros.txt"
    MAUDE_OUT="$DIRNAME/maude.txt"
    ./profile_cpp $test > $ROS_OUT
    python profile_maude.py $test > $MAUDE_OUT
    python compare.py $ROS_OUT $MAUDE_OUT
    if [ $? = "0" ]
    then
        echo "... OK"
    else 
        echo "... ERROR"
        exit
    fi
    echo
done
