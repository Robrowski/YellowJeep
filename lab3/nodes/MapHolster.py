#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import GridCells
from std_msgs.msg import Header
from geometry_msgs.msg import Point

# MapHolster Holds a map and encapsulates grid cell abstraction
class MapHolster:
    
    # Reads map meta data and saves to object constants
    def __init__(self,aMap):
        self.currentMap = aMap
        self.mapInfo = aMap.info
        self.mapOrigin  = aMap.info.origin.position 
        self.gridResolution = aMap.info.resolution # assume square map
        self.gridOrigin = Point( self.mapOrigin.x + self.gridResolution/2  , self.mapOrigin.y + self.gridResolution/2  ,  0)
    
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
        return self.currentMap.data[y*width + x]
        
        
    # Gets a list of the adjacent points that have low 
    # chances of being obstacles
    # -can also take a Point object    
    # -optional tolerance, default 100
    def getEightAdjacentPoints(self, x, y = None, tol = 100):
        # X is actually a Point object in this case
        if y == None: 
            y = x.y
            x = x.x
    
        # Make a list of the 8 adjacent points
        pointsToCheck = []
        for ax in range(x-1,x+2):
            for ay in range(y-1,y+2):
                pointsToCheck += [Point(ax,ay,0)]
        pointsToCheck.remove(Point(x,y,0)) #center
    
    
        # A list of points that are adjacent and need
        # to be checked for obstacles
        pointsWithoutObstacles = []
        for pt in pointsToCheck:
            if self.readMapPoint(pt) < tol:
                pointsWithoutObstacles += [pt]
    
        return pointsWithoutObstacles
        
    
    
