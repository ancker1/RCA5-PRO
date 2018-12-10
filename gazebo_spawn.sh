#!/bin/bash

dir=0 #Init

while getopts 'udlrx:y:t:h' option
do
   case $option in
     u) 
	up=1
	no_option=1
	echo up
     	;;
     d) 
	down=1
	no_option=1
	echo down
    	 ;;
     l)
	left=1
	no_option=1
	echo left
	;;
     r)
	no_option=1
	right=1
	echo right
	;;
     x)
	no_option=1
	manuelx=1
	posx="$OPTARG"
	;;
     y)
	no_option=1
	manuely=1
	posy="$OPTARG"
	;;
     t)
	no_option=1
	dir="$OPTARG"
	;;
     h)
	no_option=1
	help=1
	;;
     *)
	
	echo "Invalid Option. Try help with -h"
     	;;
   esac
done

#No option given
if [[ -z $no_option ]]
then
   echo "Please give an option. Try help with -h"
fi

#Help option
if [[ ! -z $help ]]
then
   cat <<- _EOF_
Usage: gazebo_spawn [OPTION]... 
Spawns the pioneer2dx in a random position and orientation
in chosen region of smallworld, or in chosen manuel position.
Also removes all marbles.

Gazebo_spawn must take and input option

  -x [INPUT],  Spawn robot in chosen x-pos, y-pos random if not set 
  -y [INPUT],  Spawn robot in chosen y-pos, x-pos random if not set 
  -t [INPUT],  Spawn robot with chosen dir, if not set = 0 [Radians]
  -u, 	       Spawn robot random in upper region, second argument either l or r 
  -d, 	       Spawn robot random in lower region, second argument either l or r 
  -l,          Spawn robot random in left region, second argument either u or d 
  -r           Spawn robot random in right region, second argument either u or d
  -h, 	       Help section
  
Examples:
  gazebo_spawn -dl  Spawn robot random i lower-left region
  gazebo_spawn -x 1 y 4  Spawn robot in chosen x-y position
_EOF_
exit 1
fi

#Generating random y position based on selected region (Up or down)
if [[ ( ! -z $up || ! -z $down ) && ( ! -z $manuely ) ]]
then
   	echo "Please only set a region or a manuel pos"
   	exit 1
elif [[ ! -z $up && -z $down ]] #Up region chosen
then
   	posy=$(( RANDOM % 4 )) #Number 0 to 4 
elif [[ ! -z $down && -z $up ]] #Down region chosen
then
  	 posy=$((( RANDOM % 5 ) - 4 ))  #Number 0 to -4
elif [[ ! -z $up && ! -z $down ]]
then
   	echo "Please only set up or down as defined region"
  	exit 1
fi

#Generating random x position based on selected region (Left or Right)
if [[ ( ! -z $left || ! -z $right ) && ( ! -z $manuelx ) ]]
then
   	echo "Please only set a region or a manuel pos"
   	exit 1
elif [[ ! -z $left && -z $right ]] #Left region chosen
then
  	 posx=$((( RANDOM % 9 ) - 6 )) #Number 2 to -6
elif [[ ! -z $right && -z $left ]] #Right region chosen
then
  	 posx=$((( RANDOM % 4 ) + 3 )) #Number 3 to 6
elif [[ ! -z $left && ! -z $right ]]
then
   	echo "Please only set left or right as defined region"
	exit 1
fi

#Choses random and/or manuel
echo $posx $posy $dir
gz model -m marble -d #Delete marbles
gz model -m pioneer2dx -x $posx -y $posy -Y $dir 
