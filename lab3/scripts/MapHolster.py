#!/usr/bin/env python

import rospy, math
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

# MapHolster Holds a map and encapsulates grid cell abstraction
class MapHolster:
    
	# Reads map meta data and saves to object constants
	def __init__(self, mapTopicName = '/map'):
		# Default stuff
		self.goal = Point(1,1,0)  # default
		self.start = Point(1,1,0) # default		
		
		# Automatic subscribers
		rospy.Subscriber(mapTopicName,  OccupancyGrid, self.mapRecieved, queue_size=None)
		rospy.Subscriber('/move_base_simple/yellowgoal',  PoseStamped, self.goalRecieved, queue_size=None)
		rospy.Subscriber('/yellowinitialpose',  PoseWithCovarianceStamped, self.startRecieved, queue_size=None)
			
    #####################################################
    ##########   Grid Cell Utilities   ##################
    #####################################################
    # makes a grid cell with the x/y indices from the map
    # can also take a Point object
	def newGridCell(self,x, y = None):
        #X is actually a Point object in this case
		if y == None:
			y = x.y
			x = x.x
    
		gx = self.gridOrigin.x + x*self.gridResolution    
		gy = self.gridOrigin.y + y*self.gridResolution
		return Point(gx, gy, 0)
      
    # Converts a list of "points" in map indices to XY grid cells
	def convertPointsToCells(self, points):
		cells = []
		for pt in points:
			cells += [self.newGridCell(pt)]
		return cells

	# converts a random cell to be the nearest map Point(whole numbers!)
	def convertCellToPoint(self, aCell):
		mx = math.trunc((aCell.x - self.gridOrigin.x + self.gridResolution/2) / self.gridResolution)
		my = math.trunc((aCell.y - self.gridOrigin.y + self.gridResolution/2) / self.gridResolution)
		return Point(mx, my,0)
		
		
    #####################################################
    ######   GridCell message Utilities   ###############
    #####################################################
     # Takes cells as a list of Points and sends them to rviz
	def makeGridCells(self, cells):
		newHeader = Header(1,rospy.get_rostime(),'map')
		return GridCells(newHeader, self.gridResolution, self.gridResolution, cells)
    
    # Helper to make GridCells message
	def makeMessage(self, points):
		return self.makeGridCells(self.convertPointsToCells(points))

   
    #####################################################
    ##########         Map Utilities   ##################
    #####################################################    
    # returns the value at the given point
    # can also take a Point object
	def readMapPoint(self, x,  y = None):
	#X is actually a Point object in this case
		if y == None: 
			y = x.y
			x = x.x
        
		width = self.currentMap.info.width    
		try:
			return self.currentMap.data[y*width + x]
		except IndexError:
			return -1 # undefined
        
    # Gets a list of the adjacent points that have low 
    # chances of being obstacles
    # -can also take a Point object    
    # -optional tolerance, default 100
	def getEightAdjacentPoints(self, x, y = None, tol = 100):
        # X is actually a Point object in this case
		if y == None: 
			y = math.trunc(x.y)
			x = math.trunc(x.x)
			
		# Make a list of the 8 adjacent points
		pointsToCheck = []
		for ax in range(x-1,x+2):
			for ay in range(y-1,y+2):
				pointsToCheck += [Point(ax,ay,0)]
		pointsToCheck.remove(Point(x,y,0)) # Center

        # A list of points that are adjacent and need
        # to be checked for obstacles
		pointsWithoutObstacles = []
		for pt in pointsToCheck:
			if self.readMapPoint(pt) < tol and self.readMapPoint(pt) != -1:
				pointsWithoutObstacles += [pt]
		return pointsWithoutObstacles
        
    
    #####################################################
    ##########   Subscriber Callbacks  ##################
    #####################################################    
	def mapRecieved(self, aMap):
		self.currentMap = aMap
		self.mapInfo = aMap.info
		self.mapOrigin  = aMap.info.origin.position 
		self.gridResolution = aMap.info.resolution # assume square map
		self.gridOrigin = Point( self.mapOrigin.x + self.gridResolution/2  , self.mapOrigin.y + self.gridResolution/2  ,  0)
	
	def goalRecieved(self, aStampedPose):
		self.goal = self.convertCellToPoint(aStampedPose.pose.position)
		
	def startRecieved(self, aPoseWithCovarianceStamped):
		self.start = self.convertCellToPoint(aPoseWithCovarianceStamped.pose.pose.position)
		print aPoseWithCovarianceStamped.pose.pose.position