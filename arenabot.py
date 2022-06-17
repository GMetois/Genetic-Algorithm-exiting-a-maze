# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

import sys
import inspyred 
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle
import numpy as np

import descartes
import shapely.geometry as sg

RADIUS = 5

def exempleIntersectionPlusieursFormes()
    a = sg.Point(-.5,0).buffer(1.)
    b = sg.Point(0.5,0).buffer(1.)

    left = a.difference(b)
    right = b.difference(a)
    middle = a.intersection(b)

    ax = plt.gca()
    ax.add_patch(descartes.PolygonPatch(left, fc='b', ec='k', alpha=0.2))
    ax.add_patch(descartes.PolygonPatch(right, fc='r', ec='k', alpha=0.2))
    ax.add_patch(descartes.PolygonPatch(middle, fc='g', ec='k', alpha=0.2))

    ax.set_xlim(-2,2)
    ax.set_ylim(-2,2)
    ax.set_aspect('equal')
    plt.show()


'''This function accepts in input a list of strings, and tries to parse them to update the position of a robot. Then returns distance from objective.'''
def fitnessRobot(listOfCommands, visualize=False) :

	# the Arena is a 100 x 100 pixel space
	arenaLength = 100
	arenaWidth = 100
	
	# let's also put a couple of walls in the arena; walls are described by a set of 4 (x,y) corners (bottom-left, top-left, top-right, bottom-right)
	walls = []

	wall1 = sg.Polygon([30,0],[40,0],[30,80],[40,80])
	wall1 = sg.Polygon([70,20],[80,20],[70,100],[80,100])

	walls.append(wall1)
	walls.append(wall2)
	
	# initial position and orientation of the robot
	startX = robotX = 10
	startY = robotY = 10
	startDegrees = 90 # 90°
	
	# position of the objective
	objectiveX = 90
	objectiveY = 90
	
	# this is a list of points that the robot will visit; used later to visualize its path
	positions = []
	positions.append( [robotX, robotY] )
	
	# TODO move robot, check that the robot stays inside the arena and stop movement if a wall is hit
	
	Degrees = startDegrees
	for (a,b) in listOfCommands :
		
		#Calcul de la nouvelle position à partir de la commande
		Degrees += a
		new_position = [positions[-1][0] + np.cos(np.radians(Degrees))*b, positions[-1][1] + np.sin(np.radians(Degrees))*a]
		
		#Vérification que l'intersection de la trajectoire et les murs est nulle
		traj = sg.LineString(positions[-1],new_position)
		collide = False
		for w in walls :
			if traj.intersection(w) != None :
				collide = True
			if collide :
				break
		#Si elle est nulle, on ajoute la position à la liste.
			if not collide :
				positions.append(new_position)
		if collide :
			break
		

	# TODO measure distance from objective
	
	last_pos = positions[-1]
	vision = sg.Point(last_pos[0],last_pos[1]).buffer(RADIUS)
	ligne_droite = sg.LineString(last_pos,[objectiveX,objectiveY])
	champ_vision = vision.area
	obstruction = 0
	for w in walls :
		obstruction += (vision.intersection(w)).area
	coeff = obstruction / champ_vision
	distanceFromObjective = (1+coeff)*ligne_droite.length
	
	# this is optional, argument "visualize" has to be explicitly set to "True" when function is called
	if visualize :
		
		figure = plt.figure()
		ax = figure.add_subplot(111)
		
		# plot initial position and objective
		ax.plot(startX, startY, 'r^', label="Initial position of the robot")
		ax.plot(objectiveX, objectiveY, 'gx', label="Position of the objective")
		
		# plot the walls
		for wall in walls :
			ax.add_patch(patches.Rectangle( (wall["x"], wall["y"]), wall["width"], wall["height"] ))
		
		# plot a series of lines describing the movement of the robot in the arena
		for i in range(1, len(positions)) :
			ax.plot( [ positions[i-1][0], positions[i][0] ], [ positions[i-1][1], positions[i][1] ], 'r-', label="Robot path" )
		
		ax.set_title("Movements of the robot inside the arena")
		ax.legend(loc='best')
		plt.show()

	return distanceFromObjective

################# MAIN
def main() :
	
	# first, let's see what happens with an empty list of commands
	listOfCommands = []
	
	generator = random.Random()
	generator.seed(42)
	
	evo = inspyred.ec.EvolutionaryComputation(generator)
	
	evo.selector = inspyred.ec.selectors.tournament_selection
	evo.variator = [inspyred.ec.variators.n_point_crossover,inspyred.ec.variators.scramble_mutation]
	evo.remplacer = inspyred.ec.remplacers.plus_remplacement
	evo.terminator = inspyred.ec.terminators.evaluation_termination
	
	listOfCommands = evo.evolve(
		generator = ,#TODO : Code it
		evaluator =  ,# TODO : code it,
		pop_size = 50,
		num_selected = 2000,
		maximize = False,
		max_evaluations = 10000,
		number_of_dimensions = 10, 
		minimum = -1,
		maximum = 1

	)
	
	
	fitnessRobot(listOfCommands, visualize=True)
	
	return 0

if __name__ == "__main__" :
	sys.exit( main() )

