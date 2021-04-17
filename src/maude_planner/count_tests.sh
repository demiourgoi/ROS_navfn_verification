#! /bin/bash

TEST_NAME=$1
NUM_MAPS=`find tests/ -name ${TEST_NAME} | wc -l`
NUM_PATHS=0

for test in $(find tests/ -name ${TEST_NAME})
do
  PATHS=`cat $test | wc -l`
  echo "$test -> $PATHS paths"
  let NUM_PATHS=NUM_PATHS+PATHS-3
done
echo
echo "TOTAL Maps: $NUM_MAPS"
echo "TOTAL Paths: $NUM_PATHS"

