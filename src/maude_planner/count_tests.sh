#! /bin/bash

NUM_MAPS=`ls tests/ -1 | wc -l`
NUM_PATHS=0

for test in $(find tests/ -name 'test*.txt')
do
  PATHS=`cat $test | wc -l`
  echo "$test -> $PATHS paths"
  let NUM_PATHS=NUM_PATHS+PATHS-3
done
echo
echo "TOTAL Maps: $NUM_MAPS"
echo "TOTAL Paths: $NUM_PATHS"

