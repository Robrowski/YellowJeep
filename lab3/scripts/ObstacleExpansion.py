#!/usr/bin/env python
import rospy, math, copy
from MapHolster import *
from YellowPublisher import *
from mapUtils import *


from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid  # map in and out


def OptimizeOccupancyGrid(data):
	global holster
	print "Deep copy... it takes time to go this deep, and I'm lazy"
	newmap = copy.deepcopy(data) #need a deeeep copy

	# Set newmap meta data
	newres = 0.20 #.2m = radius of robot
	oldRes = data.info.resolution
	newmap.info.resolution = newres
	newmap.info.width  = math.trunc(data.info.width*oldRes/newres)
	newmap.info.height = math.trunc(data.info.height*oldRes/newres) 
	w = newmap.info.width
	h = newmap.info.height 
	
	#Calculate new origin
	x = newmap.info.origin.position.x + (newres - oldRes)/2
	y = newmap.info.origin.position.y + (newres - oldRes)/2
	newmap.info.origin.position = Point(x, y,0)

	
	# condense point values
	print "Condensing map"
	newmap.data = [-1]*w*h #clear the list
	cellsPerSide = math.trunc(newres/oldRes)
	numCellsToCompress = cellsPerSide*cellsPerSide
	for x in range(w):
		for y in range(h):
			newVal = -1
			
			# convert from new map coordinates to old map coordinates
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
			
			# Set the new value in the new map
			newmap.data[y*w + x] = math.trunc(newVal)


	
	# for every point
	print "Expanding obstacles"
	expandedMap = [-1]*w*h #clear the list
 	for x in range(w):
  		for y in range(h):
  			
  			# Center point in consideration
  			midPt = newmap.data[y*w + x]
  			  			
   			try: #because i am too lazy to check for edges of the map
   				newVal = 0
   				for ax in range(x-1,x+2):
   					for ay in range(y-1,y+2):
   						# To expand obstacles, only consider the maximum probability of obstacles
  	 			 		newVal = max(newVal,newmap.data[ay*w + ax] )
  	 			
  	 			expandedMap[y*w + x] = newVal	
  	 		except IndexError:
  	 			expandedMap[y*w + x] = midPt #old value
  	 			
  			# If current point is unknown, keep it that way
  			if midPt == -1:
  				expandedMap[y*w + x] = midPt	
  	 			
	
	newmap.data = expandedMap
	
	global mapToPublish 
	mapToPublish = newmap
	print "Done making super map. Have a nice day."
	global newMapPub
	newMapPub.publish(mapToPublish)
	return newmap


if __name__ == '__main__':
	rospy.init_node('ObstacleExpander', anonymous=True)
    # Need a reference to the holster so that the map is ready
	global holster
	holster = MapHolster('/map') 
	

	rospy.Subscriber('/map',  OccupancyGrid, OptimizeOccupancyGrid, queue_size=None)

	global newMapPub
	newMapPub = rospy.Publisher('/newMap', OccupancyGrid, latch=True)


	#global mapToPublish
	#r = rospy.Rate(0.1) # 1hz
	#while not rospy.is_shutdown():
		#try:
			#newMapPub.publish(mapToPublish)
		#	print "published"
		#except NameError:
		#	pass #don't care
		#r.sleep()

	rospy.spin()
	
	
	
