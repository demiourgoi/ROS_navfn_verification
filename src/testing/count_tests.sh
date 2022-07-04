#! /bin/bash

#TEST_NAME=$1
TEST_DIR=$1
NUM_MAPS=`find ${TEST_DIR} -name test.txt | wc -l`
NUM_PATHS=0

for test in $(find ${TEST_DIR} -name test.txt)
do
  PATHS=`cat $test | wc -l`
  echo "$test -> $PATHS paths"
  let NUM_PATHS=NUM_PATHS+PATHS-3
done
echo
echo "TOTAL Maps: $NUM_MAPS"
echo "TOTAL Paths: $NUM_PATHS"

