#! /bin/bash

# Check all the tests in the tests/ directory, showing a final summary of erorrs

COUNTER=0
ERRORS=()

# Make the generated PDF reproducible by omitting the creation date
# (the Matplotlib version may also be omitted)
# (https://matplotlib.org/2.1.1/users/whats_new.html#reproducible-ps-pdf-and-svg-output)
export SOURCE_DATE_EPOCH="0"

for test in $(find tests/ -name 'test*.txt')
do
    # The width of the map
    mapWidth="$(head -n 1 $test | cut -d' ' -f1)"

    # Navigation functions are not included in the log files if the
    # map width in greater than 100
    if [ "$mapWidth" -gt "100" ]; then
        profm_args=--no-navfn
        profr_args=-n
    else
        profm_args=""
        profr_args=""
    fi

    let COUNTER=COUNTER+1
    echo "$COUNTER) Testing $test" 
    DIRNAME=$(dirname $test)
    TEST_NAME=$(basename $DIRNAME)
    ROS_OUT="$DIRNAME/ros.txt"
    MAUDE_OUT="$DIRNAME/maude.txt"
    ./profile_cpp $profr_args $test > $ROS_OUT
    python profile_maude.py $profm_args $test > $MAUDE_OUT
    rm -f ${DIRNAME}/${TEST_NAME}.pdf
    python compare.py --width="$mapWidth" $ROS_OUT $MAUDE_OUT
    if [ $? = "0" ]
    then
        echo "... OK"
    else
        ERRORS+=( "$test" )
        echo "... ERROR"
        echo "Generating potentials..."
        python compare.py --width="$mapWidth" $ROS_OUT $MAUDE_OUT --draw > /dev/null
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
