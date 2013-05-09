# Note this assumes that PKG_CONFIG_PATH is setup correctly
PLAYER_LIB=`pkg-config --cflags --libs playerc++`

common= common_functions.cc \
		cmdline_parsing.cc \
		occupancy_grid.cc \
		pathfind.cc
rel = common_functions.h \
      cmdline_parsing.h \
      lab1.h \
      occupancy_grid.h \
      pathfind.h

all: lab1

lab1: lab1.cc $(common) $(rel)
	echo Player: $(PLAYER_LIB)
	g++ -o $@ $(PLAYER_LIB) lab1.cc $(common)


clean:
	rm -f $(bin)
