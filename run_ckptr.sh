#!/bin/bash

# Runs all tests in a checkpoint
# Run as ./run_ckptr.sh <checkpoint number>

set -e
set -u

TESTSDIR=tests
CHECKPOINTPATTERN='checkpoint_'+$1

ckptr_count=`ls -al $TESTSDIR | grep $CHECKPOINTPATTERN | wc -l`

if [ "$#"  -ne "1" ] || [ $ckptr_count -eq 1 ]
then
	echo "Invalid args"
	echo "Usage: $0 <checkpoint number>"
	exit 1
fi

ckptr=$1

num_tests=`find tests/checkpoint_${ckptr}/* -maxdepth 0 -type d | wc -l`
for ((i=1; i<=$num_tests; i++))
do
	make run_test_${ckptr}_${i}
done

echo "Ran all tests"

exit 0
