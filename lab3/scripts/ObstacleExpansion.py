#!/usr/bin/env python
import rospy, math, copy, time, sys
from MapHolster import *
from YellowPublisher import *
from mapUtils import *

from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData

## Starting with the bottom left corner, shrinks the cells 
## in the given square
def cellShrinker(x, y,  cellsPerSide, maxUnknownPercent ):
	global holster
	# convert from new map coordinates to old map coordinates
	oldx = x*cellsPerSide 
	oldy = y*cellsPerSide
	
	numCellsToCompress = cellsPerSide*cellsPerSide # we compress squares
	numUnknowns = 0
	avg = 0
	
	# Find number of unknowns + average cost
	c = math.trunc(0.5*cellsPerSide)
	for mx in     range(oldx - c,  oldx + c):
		for my in range(oldy - c,  oldy + c):
			# make sure we are on the map
			if mx >= 0 and my >= 0:			
				cVal = holster.readMapPoint(mx,my)
				
				if cVal == -1:
					numUnknowns += 1
					if numUnknowns >= numCellsToCompress*maxUnknownPercent:
						return -1 # unknown!
				else:
					avg += cVal		
	return avg/(numCellsToCompress - .
			)

# Looks at a point and its neighbours and decides whether it should increase the value or not
def expandCell(x,y):
	global newmap,w,h
	# Center point in consideration
	midPt = newmap.data[y*w + x]
	 
	# If at extreme, don't change it
	if midPt == -1 or midPt == 100:	
		return midPt	 
	  			
	try: #because i am too lazy to check for edges of the map
		newVal = 0
		for ax in range(x-1,x+2): 
			for ay in range(y-1,y+2):
				# To expand obstacles, only consider the maximum probability of obstacles
		 		newVal = max(newVal,newmap.data[ay*w + ax] )
		 		
		 		#Exit early if val == 100, 100 is max
		 		if newVal == 100:
		 			return newVal 
		return  newVal	
	
	except IndexError:
		return  midPt #old value because we don't really care
			

def OptimizeOccupancyGrid(data):
	## Tuning Constants
	maxUnknownPercent = 0.50#%   # Used to speed up the condensing step
	
	global holster, newmap, newMapPub, w,h
	
	# Set newmap meta data
	newres = 0.20 #.2m = radius of robot
	oldRes = data.info.resolution
	w = math.trunc(data.info.width*oldRes/newres)
	h = math.trunc(data.info.height*oldRes/newres) 

	#Calculate new origin
	x = data.info.origin.position.x 
	y = data.info.origin.position.y 
	origin = Point(x, y,0)
	newPose = Pose(origin, data.info.origin.orientation )

	## Create  new map to send
	newHeader = Header(1,rospy.get_rostime(),'map')
	metaData = MapMetaData(rospy.get_rostime(), newres,w, h, newPose  )
	newmap = OccupancyGrid( newHeader, metaData, [])
	
	newmap.data = [-1]*w*h #clear the list
	cellsPerSide = newres/oldRes
	if cellsPerSide % 1 >= 0.5:
		cellsPerSide = int(math.ceil(cellsPerSide)) # idk... its usually 3.9999
	else:
		cellsPerSide = int(math.trunc(cellsPerSide))
		
	for x in range(w):
		for y in range(h):		
			######### Function to compress map cells
			newVal = cellShrinker(x,y, cellsPerSide, maxUnknownPercent)
			newmap.data[y*w + x] = math.trunc(newVal)
	 

	expandedMap = [-1]*w*h #clear the list
 	for x in range(w):
  		for y in range(h):
   			expandedMap[y*w + x] = expandCell(x,y  )
  	 			
	# Use the expanded obstacle map - comment this out if you want high res	
	#newmap.data = expandedMap
	
	newMapPub.publish(newmap)

### TODO: make able to do 2 nodes
if __name__ == '__main__':
	MAPISCOMINGFROM = sys.argv[1] # assume one arg
	rospy.init_node('ObstacleExpander_'  , anonymous=True)
	
	global holster
	holster = MapHolster(MAPISCOMINGFROM) 
	
	global newMapPub
	newMapPub = rospy.Publisher(MAPISCOMINGFROM + "_yellow", OccupancyGrid, latch=True)

	rospy.Subscriber(MAPISCOMINGFROM,  OccupancyGrid, OptimizeOccupancyGrid, queue_size=None)
	
	print "Ready to fix maps!"
	rospy.spin()