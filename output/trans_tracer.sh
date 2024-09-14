#!/bin/bash

# Processes the trans trace output, create .dat files for gnuplot and plot
# graph
#
# Note: This file is obselete now. Instead use SSDPlayer for visualization.

TRANS_TRACE_OUTFILE="./trans_trace.log"


# Generates a scatter plot of the write req and resp
# Helps in visualizing how write request translate into write responses
# and impact cleaning
function write_req_res {

	# Read each line

	echo "" > __write.dat

	count=0

	cat $TRANS_TRACE_OUTFILE | \
	while read line; do
		line=( $line )
		addr=${line[0]}
		typ=${line[1]}

		if [ $typ == "WriteResp" ];
		then
			echo "$count $addr 0" >> __write.dat
			((count++))
		fi

		if [ $typ == "WriteReq" ];
		then
			echo "$count $addr 1" >> __write.dat
			((count++))
		fi
	done

	echo $count

	gnuplot -persist <<-EOFMarker
		set title 'Writes Tracker'
		set ylabel 'Logical Address'
		set xlabel 'Time'
		set term png size 10000,400
		set output "trans_trace_graph.png"
		set palette defined (0 "blue", 1 "red")
		plot "__write.dat" using 1:2:3 with lines palette
	EOFMarker

}



# Helps in visualizing how write amplification changes over time
function write_amp {

	# Read each line

	echo "" > __write.dat

	writereq=0
	writeresp=0
	count=0

	cat $TRANS_TRACE_OUTFILE | \
	while read line; do
		line=( $line )
		addr=${line[0]}
		typ=${line[1]}

		if [ $typ == "WriteReq" ];
		then
			writereq=$((writereq+1))
			count=$((count+1))
			echo $count

			result=`echo $(( 100 * $writeresp / $writereq )) | sed 's/..$/.&/'`
			str[$count]="$count $result"
		fi

		if [ $typ == "WriteResp" ];
		then
			writeresp=$((writeresp+1))
			count=$((count+1))
			echo $count

			result=`echo $(( 100 * $writeresp / $writereq )) | sed 's/..$/.&/'`
			str[$count]="$count $result"
		fi
	done

	echo $count
	printf "%s\n" "${str[@]}" > __write.dat

	gnuplot -persist <<-EOFMarker
		set title 'Writes Amplification Tracker'
		set ylabel 'Write Amplification Ratio'
		set xlabel 'Time'
		set output "trans_trace_graph.png"
		plot "__write.dat" using 1:2 with lines
	EOFMarker

}

write_amp
#write_req_res
