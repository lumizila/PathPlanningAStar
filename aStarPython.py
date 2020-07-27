import sys
import cv2
import os
import numpy as np
import math
import random
from random import randint
from random import seed
import PIL
from PIL import Image, ImageDraw
from collections import namedtuple

print("Program starting...")

### NODE class
class nodeUnit:
	parentx = 0
	parenty = 0 
	f = 0 
	g = 0
	h = 0 
	x = 0 
	y = 0

	def __init__(self, parentx, parenty, f, g, h, x, y):
		self.parentx = parentx
		self.parenty = parenty
		self.f = f
		self.g = g
		self.h = h
		self.x = x
		self.y = y

	def setNode(self, parentx, parenty, f, g, h, x, y):
		self.parentx = parentx
		self.parenty = parenty
		self.f = f
		self.g = g
		self.h = h
		self.x = x
		self.y = y

 	
### PRINTING PATH ON IMG
def reconstructPath(nodes, finalNode, minScaleMap):
	path = []	
	#path.append([finalNode.x*scale, finalNode.y*scale])
	currentNode = finalNode

	while(currentNode.x != 0 or currentNode.y != 0):
		for n in nodes: #nodes has all nodes of the map, including black ones
			if(n.x == currentNode.parentx and n.y == currentNode.parenty):
				currentNode = n
				if(n.x == 0 and n.y == 0):
					break
				path.append([currentNode.x*scale, currentNode.y*scale])

	print("success, goal reached")
	print("printing path...")
	pathMap = np.zeros((hw, hw, 3), dtype=np.uint8)


	for i in range(0, 50):
		scaledI = i*scale
		for j in range(0, 50):
			scaledJ = j*scale
			if(minScaleMap[i][j] == 0):
				pathMap[scaledI:scaledI+scale, scaledJ:scaledJ+scale] = white
			elif(minScaleMap[i][j] == 3):
				pathMap[scaledI:scaledI+scale, scaledJ:scaledJ+scale] = black
			elif(minScaleMap[i][j] == 2):
				pathMap[scaledI:scaledI+scale, scaledJ:scaledJ+scale] = blue
			elif(minScaleMap[i][j] == 1):
				pathMap[scaledI:scaledI+scale, scaledJ:scaledJ+scale] = babyBlue
			else:
				pathMap[scaledI:scaledI+scale, scaledJ:scaledJ+scale] = green
			
	#painting beggining and end with different colors
	pathMap[0:scale, 0:scale] = pink
	pathMap[hw-scale:hw, hw-scale:hw] = pink

	##first frame
	img = Image.fromarray(pathMap, 'RGB')
	os.system("rm images/*")
	img.save('images/img00.png')

	imageOrder = 0
	path.reverse()
	for n in path:
		#print(n)
		pathMap[n[0]:n[0]+scale, n[1]:n[1]+scale] = red
		img = Image.fromarray(pathMap, 'RGB')
		imageOrder = imageOrder + 1
		if (imageOrder < 10):
			img.save('images/img0{}.png'.format(str(imageOrder)))
		else:
			img.save('images/img{}.png'.format(str(imageOrder)))
		
	
	image_folder = 'images'
	video_name = 'video.avi'

	images = [img for img in os.listdir(image_folder) if img.endswith(".png")]

	images.sort()
	frame = cv2.imread(os.path.join(image_folder, images[0]))
	height, width, layers = frame.shape

	video = cv2.VideoWriter(video_name, 0, 3, (width,height))

	for image in images:
	    video.write(cv2.imread(os.path.join(image_folder, image)))

	cv2.destroyAllWindows()
	video.release()
	
	print("video created, check local folder")
	return

def distanceToGoalHeuristic(i,j, distType):
	#using diagonal distance
	if(distanceType == "d"):
		if(abs(i-50) < abs(j-50)):
			distanceToGoal = abs(j-50)
		else:
			distanceToGoal = abs(i-50)

	#using manhatan distance
	if(distanceType == "m"):
		distanceToGoal =  abs(j-50) + abs(i-50)

	return distanceToGoal

def distanceToGoalHeuristicWeight(i,j, color, distType):
	#using diagonal distance
	if(distanceType == "d"):
		if(abs(i-50) < abs(j-50)):
			distanceToGoal = abs(j-50)
		else:
			distanceToGoal = abs(i-50)

	#using manhatan distance
	if(distanceType == "m"):
		distanceToGoal =  abs(j-50) + abs(i-50)
     
	#assume node with infinite weight at first
	weight = math.inf

	if(color == 0): #white
		weight = 10
	if(color == 1): #baby blue
		weight = 20
	if(color == 2): #dark blue
		weight = 30
	return distanceToGoal*weight

###DEFINING IMAGE SIZE
heightWidthStr = input("---> What is the height/width of image in pixels? (must me multiple of 50) ")
hw = int(heightWidthStr)
scale = int(hw/50)
print("The scale of the image is: "+str(scale))

###DEFINING COLORS TO REPRESENT WEIGHT
red = [250, 10, 10] 
pink = [255, 0, 255] 
blue = [0, 0, 255] 
babyBlue = [0, 255, 255] 
black = [0, 0, 0] # Infinity
white = [255, 255, 255] #0
green = [128, 255, 0] # used for error

###CREATING THE INITIAL MAP WITH RANDOM SEED
seedStr = input("---> What is the seed you want to use for the random map? ")
seed(int(seedStr))

###CREATING MATRIX WITH RANDOM VALUES for black(blocked = 3)and white(open = 0)
minScaleMap = np.zeros((50, 50), dtype=np.uint8)
for i in range(0,50):
	for j in range(0,50):
		minScaleMap[i][j] = random.choice((0,0,0,0,0,3)) ##making white more likely than black

weightStr = input("---> Do you want more weight near barriers (answer yes or no)? ")
if(weightStr != "yes" and weightStr != "no"):
	print("Error: answer not known")
	exit()

distanceType = input("---> Press >m< for manhatan distance, >d< for diagonal distance when measuring distance to goal: ")
if(distanceType != "m" and distanceType != "d"):
	print("Error: answer not known")
	exit()

elif(weightStr == "yes"):
	###Assigning values according to proximity to black -> blue(open, next to black = 2), baby blue(open, next to black = 1)
	for i in range(0,50):
		for j in range(0,50):
			#if this node is not black
			if(minScaleMap[i][j] != 3):
				for neighborI in range (i-2, i+3):
					for neighborJ in range (j-2, j+3):
						#checking if coordinates are not out of map and not the same as i and j 
						if(neighborI >=0 and neighborJ >= 0 and (neighborI != i or neighborJ != j) and neighborI < 50 and neighborJ < 50):
							#assigning light blue
							if(minScaleMap[neighborI][neighborJ] == 3 and (abs(i-neighborI) == 2 or abs(j-neighborJ) == 2)):
								#if its white you can paint
								if(minScaleMap[i][j] == 0):
									minScaleMap[i][j] = 1
							#assigning blue
							elif(minScaleMap[neighborI][neighborJ] == 3):
								minScaleMap[i][j] = 2

###Start and finish must not be blocked
minScaleMap[0][0] = 0
minScaleMap[49][49] = 0

###Applying A* Search

nodes = [nodeUnit(None,None,None,None,None,None,None) for i in range(50*50)]

for i in range(0,50):
	for j in range(0,50):
		#For node n, g is the cost of the cheapest path from start to n currently known.
		#f represents our current best guess as to how short a path from start to finish can be if it goes through n.
		nodes[i*50 + j].setNode(-1, -1, math.inf, math.inf, math.inf, i, j)

#initializing starting node
nodes[0] = nodeUnit(-1,-1,0,0,0,0,0)

#Creating open list
openList = []
#Creating closed list, which will show the nodes already searched 
closedList = np.zeros((50, 50), dtype=bool)
for i in range(0,50):
	for j in range(0,50):
		closedList[i][j] = False

#adding starting node to openList with f=0
openList.append(nodes[0])
foundGoal = False

while(openList):

	##finding node in openlist with min f value
	minfNode = math.inf
	for i in range(0, len(openList)):
		if(openList[i].f < minfNode):
			minfNode = openList[i].f;
			chosenNode = i; 
	##testing if it found a node possible to pass in openList, if not it means there is no open path
	if(minfNode != math.inf):
		##defining it as current ##removing form openlist
		currentNode = openList.pop(chosenNode)
		closedList[currentNode.x][currentNode.y] = True

		#for each successor of current node
		for i in range(currentNode.x-1, currentNode.x+2):
			#making sure its inside range
			if(i >= 0 and i < 50):
				for j in range(currentNode.y-1, currentNode.y+2):
					#making sure its inside range and not same as parent node
					if(j >= 0 and j < 50 and not(currentNode.x == i and currentNode.y == j)):
						#Boolean indicating if this is an acceptable successor to the current Node
						thisNodeisOk = True
						
						#if this successor node is the goal
						if(nodes[i*50 + j].x == 49 and nodes[i*50 + j].y == 49):
							foundGoal = True
							nodes[i*50 + j].parentx=currentNode.x 
							nodes[i*50 + j].parenty=currentNode.y
							reconstructPath(nodes, nodes[i*50 + j], minScaleMap)
							exit()

						#testing if this is top right corner successor
						if(i == currentNode.x-1 and j == currentNode.y+1):

							#if the sides of this corner successor are black, you cant pass
							if(minScaleMap[i][j-1] == 3 and minScaleMap[i+1][j] == 3):    
								thisNodeisOk = False

						#testing if this is top left corner successor
						if(i == currentNode.x-1 and j == currentNode.y-1):

							#if the sides of this corner successor are black, you cant pass
							if(minScaleMap[i+1][j] == 3 and minScaleMap[i][j+1] == 3): 
								thisNodeisOk = False

						#testing if this is bottom left corner successor
						if(i == currentNode.x+1 and j == currentNode.y-1):

							#if the sides of this corner successor are black, you cant pass
							if(minScaleMap[i-1][j] == 3 and minScaleMap[i][j+1] == 3):  
								thisNodeisOk = False

						#testing if this is bottom right corner successor
						if(i == currentNode.x+1 and j == currentNode.y+1):

							#if the sides of this corner successor are black, you cant pass
							if(minScaleMap[i-1][j] == 3 and minScaleMap[i][j-1] == 3): 
								thisNodeisOk = False

						#checking if this node is on closedList already
						if(closedList[i][j] == True):
							thisNodeisOk = False

						#making sure this successor is not black or blocked or already in the closed list
						if(minScaleMap[i][j] != 3 and thisNodeisOk):

							g = currentNode.g + 1

							if(weightStr == "yes"):
								h = distanceToGoalHeuristicWeight(i,j, minScaleMap[i][j],distanceType)
							else:
								h = distanceToGoalHeuristic(i,j,distanceType)
								
							f = h + g

							if((nodes[i*50 + j] not in openList) or (f < nodes[i*50 + j].f)):
								nodes[i*50 + j].g=g
								nodes[i*50 + j].h=h
								nodes[i*50 + j].f=f
								nodes[i*50 + j].parentx=currentNode.x 
								nodes[i*50 + j].parenty=currentNode.y 
								openList.append(nodes[i*50 + j])
	else:
		break
if(foundGoal == False):
	print("Failure: goal unreacheable")


