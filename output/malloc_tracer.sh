#!/bin/bash

# Processes the malloc trace output, create .dat files from gnuplot and plot
# graph


# Stack trace output
MALLOC_TRACE_OUTFILE="./malloc_trace.dat"

if [ ! -f $MALLOC_TRACE_OUTFILE ]; then
    echo "$MALLOC_TRACE_OUTFILE File not found."
    exit 1
fi

#############
# Plot graph
#############

# If too many points to display, xticks becomes clobbered due to being too
# close

gnuplot -persist <<-EOFMarker
	set title 'Heap Usage Tracker'
	set ylabel 'Heap Usage(bytes)'
	set ytics nomirror textcolor lt 1
	set xtics rotate font "Times-Roman, 6"
	plot \
		"$MALLOC_TRACE_OUTFILE" using 1:xtic(2) with line title 'Cumulative Heap Change' linetype 1
	if (GPVAL_ERRNO != 0) exit
	# Save the graph to file
	set term png
	set output "malloc_trace_graph.png"
	replot
	set term xterm
EOFMarker
