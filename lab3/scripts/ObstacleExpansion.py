#!/usr/bin/env python
import rospy, math, copy
from MapHolster import *
from YellowPublisher import *
from mapUtils import *


from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid  # map in and out


def OptimizeOccupancyGrid(map):
	holster = MapHolster()
	print "Deep copy... it takes time to go this deep, and I'm lazy"
	newmap = copy.deepcopy(map) #need a deeeep copy
	

	# Set newmap meta data
	newres = 0.20 #.2m = radius of robot
	newmap.info.resolution = newres
	newmap.info.width = math.trunc(map.info.resolution/newres*map.info.width)
	newmap.info.height = math.trunc(map.info.resolution/newres*map.info.height) 
	w = newmap.info.width
	h = newmap.info.height 
	
	#Calculate new origin
	x = newmap.info.origin.position.x + (newres - map.info.resolution)/2
	y = newmap.info.origin.position.y + (newres - map.info.resolution)/2
	newmap.info.origin.position = Point(x, y,0)

	
	# condense point values
	print "Condensing map"
	newmap.data = [-1]*w*h #clear the list
	cellsPerSide = math.trunc(newres/map.info.resolution)
	numCellsToCompress = cellsPerSide*cellsPerSide
	for x in range(w):
		for y in range(h):
			newVal = -1
			
			oldx = x*cellsPerSide 
			oldy = y*cellsPerSide
			avg = 0
			numUnknowns = 0
			
			# Find number of unknowns + average cost
			for mx in range(oldx,oldx+cellsPerSide):
				for my in range(oldy,oldy+cellsPerSide):
					cVal = holster.readMapPoint(mx,my)
					if cVal == -1:
						numUnknowns += 1
					else:
						avg += cVal/numCellsToCompress;	
			
			# If too much unknown, set cell to unknown
			if numUnknowns >= numCellsToCompress*0.75:
				newVal = -1
			else:
				newVal = avg			
			
			newmap.data[y*w + x] = math.trunc(newVal)
	
	# for every point
	print "Expanding obstacles"
	expandedMap = [-1]*w*h #clear the list
 	for x in range(w):
  		for y in range(h):
  			midPt = newmap.data[y*w + x]
  			  			
   			try: #because i am too lazy to check for walls
   				newVal = 0
   				for ax in range(x-1,x+2):
   					for ay in range(y-1,y+2):
  	 			 		newVal = max(newVal,newmap.data[ay*w + ax] )
  	 			
  	 			expandedMap[y*w + x] = newVal	
  	 		except IndexError:
  	 			expandedMap[y*w + x] = midPt #old value
  	 			
  			# If current point is unknown, keep it that way
  			if midPt == -1:
  				expandedMap[y*w + x] = midPt	
  	 			
	
	newmap.data = expandedMap
	
	print "Publishing"
	rospy.Publisher('/newMap', OccupancyGrid).publish(newmap)
	print "Done Publishing"

if __name__ == '__main__':
	rospy.init_node('ObstacleExpander', anonymous=True)
    # Need a reference to the holster so that the map is ready
	global h
	h = MapHolster() 
	
	rospy.Subscriber('/map',  OccupancyGrid, OptimizeOccupancyGrid, queue_size=None)

	# Put cells that are probably obstacles into list
	obstacles = []
	
	# for every point
#	for x in range(h.mapInfo.width):
# 		print "workin on it..."
# 		YellowPublisher().sendToObstacles(obstacles)
# 		for y in range(h.mapInfo.height):
# 			neighbors = h.getEightAdjacentPoints(x,y,999) #huge tolerance
# 			for pt in neighbors:
# 				if h.readMapPoint(pt) > 0 or h.readMapPoint(pt) == -1:
# 					obstacles.append(Point(x,y,0))
# 					break;

	
	
# 	i = 0
# 	while (1 < 9000):
# 		YellowPublisher().sendToObstacles(obstacles)
# 		i += 1
# 	
	rospy.spin()
	
	
	