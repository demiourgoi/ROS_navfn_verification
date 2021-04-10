#! /bin/bash

# Produces ROS and Maude output and check the results for one test
# If the comparison fails, generates the potentials

test=$1
DIRNAME=$(dirname $test)
ROS_OUT="$DIRNAME/ros.txt"
MAUDE_OUT="$DIRNAME/maude.txt"

# Make the generated PDF reproducible by omitting the creation date
# (the Matplotlib version may also be omitted)
# (https://matplotlib.org/2.1.1/users/whats_new.html#reproducible-ps-pdf-and-svg-output)
export SOURCE_DATE_EPOCH="0"

# Navigation functions are not included in the log files if the
# map width in greater than 100
if [ "$(head -n 1 $test | cut -d' ' -f1)" -gt "100" ]; then
  profm_args=--no-navfn
  profr_args=-n
else
  profm_args=""
  profr_args=""
fi

./profile_cpp $profr_args $test > $ROS_OUT
python profile_maude.py $profm_args $test > $MAUDE_OUT
python compare.py $ROS_OUT $MAUDE_OUT
if [ $? = "0" ]
then
  echo "... OK"
  exit 0
else
  echo "... ERROR. Generating potentials:"
  TEST_NAME=$(basename $DIRNAME)
  python compare.py $ROS_OUT $MAUDE_OUT --draw > /dev/null
  mv potentials.pdf ${DIRNAME}/${TEST_NAME}.pdf
  exit -1
fi
