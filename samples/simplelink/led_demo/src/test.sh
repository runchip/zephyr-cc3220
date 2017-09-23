#!/bin/bash

function test_once()
{
	sleep 1
	echo -e '\x04\x00\x00\x00\x4D\x3C\x2B\x1AABCD\c' | nc localhost 9090 | hd
}

for ((i=0; i<10; i++)); do
	test_once &
done


