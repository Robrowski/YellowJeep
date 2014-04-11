#!/usr/bin/env python
import rospy, math, copy
from MapHolster import *
from YellowPublisher import *
from mapUtils import *


from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid  # map in and out


def OptimizeOccupancyGrid(map):
	h = MapHolster()
	print "Deep copy... it takes time to go this deep"
	newmap = copy.deepcopy(map) #need a deeeep copy
	

	# Set newmap meta data
	newres = 0.20 #.2m = radius of robot
	newmap.info.resolution = newres
	newmap.info.width = math.trunc(map.info.resolution/newres*map.info.width)
	newmap.info.height = math.trunc(map.info.resolution/newres*map.info.height) 
	
	#Calculate new origin
	x = newmap.info.origin.position.x + (newres - map.info.resolution)/2
	y = newmap.info.origin.position.y + (newres - map.info.resolution)/2
	newmap.info.origin.position = Point(x, y,0)

	
	# condense point values
	print "Condensing map"
	newmap.data = [-1]*newmap.info.height*newmap.info.width #clear the list
	cellsPerSide = math.trunc(newres/map.info.resolution)
	numCellsToCompress = cellsPerSide*cellsPerSide
	for x in range(newmap.info.width):
		for y in range(newmap.info.height):
			newVal = -1
			
			oldx = x*cellsPerSide 
			oldy = y*cellsPerSide
			avg = 0
			numUnknowns = 0
			
			for mx in range(oldx,oldx+cellsPerSide):
				for my in range(oldy,oldy+cellsPerSide):
					cVal = h.readMapPoint(mx,my)
					if cVal == -1:
						numUnknowns += 1
					else:
						avg += cVal/numCellsToCompress;	
			
			if numUnknowns == numCellsToCompress:
				newVal = -1
			else:
				newVal = avg			
			
			print newVal
			newmap.data[y*newmap.info.width + x] = math.trunc(newVal)
	
	print newmap.data
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
	
	
	