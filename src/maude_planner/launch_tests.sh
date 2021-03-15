#! /bin/bash

# Check all the tests in the tests/ directory, showing a final summary of erorrs

COUNTER=0
ERRORS=()

for test in $(find tests/ -name 'test*.txt')
do
    let COUNTER=COUNTER+1
    echo "$COUNTER) Testing $test" 
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
        ERRORS+=( "$test" )
        echo "... ERROR"
        echo "Generating potentials..."
        TEST_NAME=$(basename $DIRNAME)
        python compare.py $ROS_OUT $MAUDE_OUT --draw
        mv potentials.pdf ${DIRNAME}/${TEST_NAME}.pdf
        # exit
    fi
done

FAILED=0
echo
echo "Test that failed:"
for t in ${ERRORS[@]}; do
  let FAILED=FAILED+1
  echo "* $t"
done
echo "Failed ${FAILED}/${COUNTER}"
