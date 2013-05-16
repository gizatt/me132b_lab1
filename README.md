me132b_lab1
===========

ME132b Lab 1 Repository for Team [Matthew Dughi, Tiffany Huang, Gregory Izatt]

Code base for navigating a Pioneer2 (with laser scanner) around a 
moderate-sized obstacle in an otherwise spacious room, using player/stage 
for interface (and simulation).

Project status:
Successfully (or perhaps "successfully") demo'd to Mary on 20130514.

BUILD AND TEST INSTRUCTIONS:
============================

Making is as simple as:
"make"
(though "make clean" to clear things out on your end first doesn't
hurt)

Running in simulation:
launch player with "player -p <port> ./configs/lab.cfg"
launch robot with "./lab1 -p <port>""

Running in lab:
launch player on Honolulu (or equiv) with
"player ~/configs/pioneer2_laser.cfg" (or something like that)
launch robot with "./lab1"

Note that both of these sometimes don't maange to connect or
initialize quite right -- operation at that point is pretty
much undefined. All it takes is trying again to get things to
work, usually.


INCLUDED FILES:
===============

All files that we've done a lot of work in have extensive file and
function headers that should provide adequate guidance, but at the
highest level, here's how things work:

lab1.cc is the main loop, which manages connections to player/stage,
extracting sensor data, calling functions to update occupancy map and
do navigation.

lab1.h includes control constants for pretty much every part of the
project. Specifically relevant to the demo are DANGER_MAX_THRESH and 
TRAVERSE_MAX_THRESH, which dictate the closest and farthest distances we're
okay navigating from an obstacle; modifications to just those are are all
that's necessary to change navigating around the gap, and through it,
in the demo setup we were given~

cmdline_parsing.cc,.h, and common_functions.cc,.h were supplied to us
in ME132a labs, and just handle routine parts of dealing with player/
stage and such.

occupancy_grid.cc/.h: Manages occupancy grid. A shell of this file
was provided, but we've expanded it quite liberally: it now does
c-space approximation, console-based (ASCII-based) rendering of the
entire (or just local) occupancy grid, and has a lot of helpers for
navigation. (See far below for an example of these renderings
and some guidance on how to interpret them.)

pathfind.cc/.h: Manages pathfinding / navigation. Manages a state
machine deciding the robot's current subgoal at any moment, which
calls a range of helper functions to do different parts of the
navigation -- the file header is particularly lengthy on this
one ;)

Makefile -- makes... files.

./configs: contains various files for running a simulation. The
included world (lab.png) is good for practicing the close-follow
(going through narrow gaps) mode of operation, but doesn't really
work for the other mode.

./unused_files: old test files and things that we used in
development but aren't part of the core suite.

./misc_logs: logs that we've help on to in case we care to
look at them later. Logs currently aren't very informative --
most of the important info just goes to console. (This should
be remedied some time in the future, but presently problems
are short-term enough (particular cases failing that we can
see and deal with immediately) that a log isn't terribly
useful or worth the effort to set up and read.)



EXAMPLE OUTPUT OF OCCUPANCY GRID RENDERING
==========================================

As discussed in occupancy.cc, the occupancy grid (and corresponding
c-grid) can be rendered to the console at any moment via helper functions.
The rendering is oriented the same way the player sim is -- up is +y,
down is -y, left is -x, right is +x. 

Obstacles are rendered as "8", "+", or "-", in decreasing order of
certainty that the obstacle is truly in that cell.

The c-grid, if rendered, is shown with "!" in all cells that we've
determined are too close to an obstacle to want to navigate through,
and with "#" in the ones that are just the right distance away -- not
too close, not too far. (These are based on the *_THRESH constants
we mention above).

Implicitely, then, unlabeled space has no obstacle, but is not traversable,
as it is too far away from an obstacle. (We want to do wall following,
not room exploring!)

We also show the location of the robot with an "X" if it's within
the view, and the location of its goal (if applicable -- this is
only done in the local view) if within view.

There are two helper functions that do two different kinds of
rendering. One renders the whole occupancy grid, downsampling to
the requested display resolution by taking the most salient
feature in each block -- dangerous space or obstacles
take priority over traversable space, which takes precedence
over free space. The other renders just local space, showing the
requested number of cells horizontally and vertically, using
the same demarcations, but not requiring any downsampling.

Example renderings:

Global:
'



                       ##################################
                     ###!!!!!!!!!!!!!!!#!!!!!!!!!!!!!!!!###
                     ##!!88888888888888888888888888888!!!!###
                     ##!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!8888!!##
                      #############################!!!!!8!!##
                                    ###### ###### ###########
                                  ###!!!!###!!!!### ########
                              ######!!8!!##!!88!!####!!!!!###
                             ###!!!!!!8!!#X!!88!!!!!!!!88!!###
                             ###!!88888!!##!!888888!!!!!8!!###
                             ###!!8!!#################!!8!!###
                             ###!!8!!##            ###!!88!!##
                             ###!!8!!##           ##!!!8!8!!##
                             ###!!8!!##          ###!!88!8!!##
                             ###!!8!!##          ###!!8!!!!###
                             ###!!!!!##          ##!!88!!###
                         ############# ######### ##!!88!!##
                      ####!!!###########!!!!!!!####!!+8!!###
                      ###!!8!!!!!!!!!!!!!8888888!!!!!88!!###
                      ##!!8888888888888888!!!!!!888888!!###
                      ##!!!!!!!!!!!!!!!!!!!!##!!!!!!!!!####
                       #################################



'
A global rendering of the robot currently positioned between
two obstacles. You can see the L-shaped primary obstacle
on the left, and a square obstacle on the right. You can also
see some of the walls of the room, but not quite all of them. Note that 
the robot is presently in free space. Also note that the robot 
can't see what's behind the obstacles on either side, so it claims 
there's free space behind there when there really might not be. 
Due to the way our algorithm always paths only to somewhere within 
the line of sight, this is OK -- we'll update the map over there
before we try to drive into it.


Local:
'
                            #########     ##########          ###########
                           ###########   ############          #########
                         ##############################
                         #################O############
                        ################################      #########
                       #######!!!!!#########!!!!!!#######    ###########
                       ######!!!!!!!#######!!!!!!!!######  ###############
                       #####!!!!!!!!!#####!!!!!!!!!!#####  ###############
                   #########!!!!!!!!!#####!!!!!!!!!!#######################
                  ##########!!!!8!!!!#####!!!!+8!!!!############!!!!!#######
                ############!!!!8!!!!#####!!!!88!!!!###########!!!!!!!######
                ############!!!!8!!!!#####!!!!88!!!!##########!!!!!!!!!#####
               #############!!!!8!!!!#####!!!!88!!!!##########!!!!!!!!!######
              #######!!!!!!!!!!!8!!!!#####!!!!88!!!!!!!!!!!###!!!!8!!!!#######
              ######!!!!!!!!!!!!8!!!!#####!!!!88!!!!!!!!!!!!##!!!!8!!!!!######
              #####!!!!!!!!!!!!!8!!!!##X##!!!!88!!!!!!!!!!!!!#!!!!8!!!!!!#####
              #####!!!!!!!!!!!!!8!!!!#####!!!!88!!!!!!!!!!!!!#!!!!88!!!!!#####
              #####!!!!8888888888!!!!#####!!!!88888888888!!!!#!!!!!88!!!!#####
              #####!!!!8!!!!!!!!!!!!!#####!!!!!-!!!!!!!!!!!!!#!!!!!88!!!!#####
              #####!!!!8!!!!!!!!!!!!!#####!!!!!!!!!!!!!!!!!!!##!!!!88!!!!#####
              #####!!!!8!!!!!!!!!!!!#######!!!!!!!!!!!!!!!!!###!!!!!8!!!!#####
              #####!!!!8!!!!!!!!!!!#########!!!!!!!!!!!!!!!####!!!!!8!!!!#####
              #####!!!!8!!!!#################!!!!!#############!!!!!8!!!!#####
              #####!!!!8!!!!###################################!!!!!8!!!!#####
              #####!!!!8!!!!###################################!!!!+8!!!!#####
              #####!!!!8!!!!##########  #######################!!!!88!!!!#####
              #####!!!!8!!!!#########     #####################!!!!88!!!!#####
              #####!!!!8!!!!#####          #########      #####!!!!8!!!!!!####
              #####!!!!8!!!!#####                         #####!!!!88!!!!!####
              #####!!!!8!!!!#####                         #####!!!!888!!!!####
              #####!!!!8!!!!#####                        ######!!!!!!8!!!!####
'
A local rendering after the robot has traversed between the two
obstacles, scanned, as is planning on moving up to a point
outside of them. Note that the choice of path we've made
doesn't require the robot to cross illegal space.