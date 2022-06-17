# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>

from shutil import move
import sys
import inspyred 
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle
import numpy as np

import descartes
import shapely.geometry as sg

from enum import Enum

## CONSTANTS

RADIUS = 5

WALLS = []

ARENA_LIMITS = sg.LinearRing([(0,0),(0,100),(100,100),(100,0)])
WALLS.append(sg.Polygon([(30,0),(40,0),(40,80),(30,80)]))
WALLS.append(sg.Polygon([(70,20),(80,20),(80,100),(70,100)]))

OBJECTIVE = sg.Point(90,90)
INITIAL_POSITION = sg.Point(10,10)
INITIAL_ANGLE = 90 # en °

## Codage des instructions
class MoveType(Enum):
    FORWARD = 1
    ROTATE = 2

class Instruction():
	def __init__(self,moveType:MoveType,amplitude:float):
		self.amplitude = amplitude
		self.moveType = moveType
    
	def applyInstruction(self,point:sg.Point,orientation:float):
		if self.moveType == MoveType.ROTATE:
			return (point,(orientation+self.amplitude)%360)
		elif self.moveType == MoveType.FORWARD:
			a = point.x+np.cos(np.radians(orientation))*self.amplitude 
			b = point.y+np.sin(np.radians(orientation))*self.amplitude
			return (sg.Point(a,b),orientation)

	def applyInstructions(instructions, initialPoint:sg.Point, initialAngle:float):
		pointList=[initialPoint]
		angleList=[initialAngle]
		for instruction in instructions:
			newPoint,newAngle=instruction.applyInstruction(pointList[-1],angleList[-1])
			pointList.append(newPoint)
			angleList.append(newAngle)
		return (pointList,angleList)
	
	def applyInstructionsUntilHit(instructions,initialPoint:sg.Point,initialAngle:float):
		positions = [initialPoint]
		angles = [initialAngle]
		for instruction in instructions :
			#Calcul de la nouvelle position à partir de la commande
			newState= instruction.applyInstruction(positions[-1],angles[-1])
			positions.append(newState[0])
			angles.append(newState[1])
			#Vérification que l'intersection de la trajectoire et les murs est nulle
			traj = sg.LineString([positions[-1],positions[-2]])
			collide = not(traj.intersection(ARENA_LIMITS))
			for w in WALLS :
				if not (traj.intersection(w).is_empty) or collide:
					collide = True
					break
			
			#Si elle est nulle, on ajoute la position à la liste.
			if collide :
				break
		return (positions, angles)

	''' Représentation sous forme de chaine de caractère d'une instruction : F100 ou R90 (avance de 100 ou tourne de 90). Peut être appelé par str(Instruction(MoveType.ROTATE,10))'''
	def __str__(self) -> str:
		moveTypeString = "ERR"
		if self.moveType == MoveType.ROTATE:
			moveTypeString = "R"
		elif self.moveType == MoveType.FORWARD:
			moveTypeString = "F"
		return moveTypeString+str(self.amplitude)
	
	def __repr__(self) -> str:
		return str(self)

## Heuristiques et évaluateurs pour inspyred

def heuristic_1(lastPos:sg.Point):
	vision = lastPos.buffer(RADIUS)
	ligne_droite = sg.LineString([lastPos,OBJECTIVE])
	champ_vision = vision.area
	obstruction = 0
	for w in WALLS :
		obstruction += (vision.intersection(w)).area
	coeff = obstruction / champ_vision
	return (1+coeff)*ligne_droite

def evaluator_1(candidates,args):
	out = []
	for candidate in candidates:
		states= Instruction.applyInstructionsUntilHit(candidate,INITIAL_POSITION,INITIAL_ANGLE)
		out.append(heuristic_1(states[0][-1]))
	return out

def heuristic_2(lastPos:sg.Point):
	ligne_droite = sg.LineString([lastPos,OBJECTIVE])
	return ligne_droite.length

def evaluator_2(candidates,args):
	out = []
	for candidate in candidates:
		states= Instruction.applyInstructionsUntilHit(candidate,INITIAL_POSITION,INITIAL_ANGLE)
		wCollisions = Instruction.applyInstructions(candidate,INITIAL_POSITION,INITIAL_ANGLE)
		out.append((1+len(wCollisions)-len(states))*heuristic_2(states[0][-1]))
	return out

def generator(random,args):
	actual_number_of_moves = random.randint(args["min_number_of_moves"],args["max_number_of_moves"])
	individual = []
	for y in range(actual_number_of_moves):
		if individual == []:
			moveType = random.choice([MoveType.FORWARD,MoveType.ROTATE])
		else: 
			if individual[-1].moveType == MoveType.FORWARD:
				moveType = MoveType.ROTATE
			else:
				moveType = MoveType.FORWARD
		
		if moveType == MoveType.FORWARD:
			amplitude = random.randint(0,args["max_deplacement_amplitude"])
		elif moveType == MoveType.ROTATE:
			amplitude = random.randint(0,args["max_rotation_amplitude"])*random.choice([-1,1])
		
		individual.append(Instruction(moveType,amplitude))
	return individual

'''This function accepts in input a list of strings, and tries to parse them to update the position of a robot. Then returns distance from objective.'''
def fitnessRobot(listOfCommands, visualize=False) :
	# this is a list of points that the robot will visit; used later to visualize its path
	positions,angles = Instruction.applyInstructionsUntilHit(listOfCommands,INITIAL_POSITION,INITIAL_ANGLE)
	distanceFromObjective = heuristic_2(positions[-1])
	
	# this is optional, argument "visualize" has to be explicitly set to "True" when function is called
	if visualize :
		display(listOfCommands)

	return distanceFromObjective

def display(instructions): 
	figure = plt.figure()
	ax = figure.add_subplot(111)
	
	# plot initial position and objective
	ax.plot(INITIAL_POSITION.x, INITIAL_POSITION.y, 'r^', label="Initial position of the robot")
	ax.plot(OBJECTIVE.x, OBJECTIVE.y, 'gx', label="Position of the objective")
	
	# plot the walls
	for wall in WALLS :
		# ax.add_patch(patches.Rectangle( (wall["x"], wall["y"]), wall["width"], wall["height"] ))
		xs,ys = wall.exterior.xy
		ax.fill(xs,ys,alpha=0.5,fc='b',ec='none')
	# plot a series of lines describing the movement of the robot in the arena

	#Plot de la trajectoire sans collision	
	pointsWithoutColision = Instruction.applyInstructions(instructions,INITIAL_POSITION,INITIAL_ANGLE)[0]
	lineWithoutColision=sg.LineString(pointsWithoutColision)
	colisionFreeX,colisionFreeY = lineWithoutColision.xy
	ax.plot(colisionFreeX,colisionFreeY, 'r--', label="Robot path without colision" )

	#Plot de la véritable trajectoire (avec collision)
	positions = Instruction.applyInstructionsUntilHit(instructions,INITIAL_POSITION,INITIAL_ANGLE)[0]
	line = sg.LineString(positions)
	lx,ly = line.xy
	ax.plot(lx,ly, 'r-', label="Robot path" )
	
	# Limites de l'arène
	xLimits,yLimits=ARENA_LIMITS.xy
	ax.plot(xLimits,yLimits,'b-',label='Arena limits')

	ax.set_title("Movements of the robot inside the arena")
	ax.legend(loc='best')
	plt.show()

################# MAIN
def main() :
	
	# first, let's see what happens with an empty list of commands
	# listOfCommands = [Instruction(MoveType.FORWARD,90),Instruction(MoveType.ROTATE,-90),Instruction(MoveType.FORWARD,40),Instruction(MoveType.ROTATE,-90),Instruction(MoveType.FORWARD,40)]
	listOfCommands = []
	random_generator = random.Random()
	random_generator.seed(42)
	
	evo = inspyred.ec.EvolutionaryComputation(random_generator)
	
	evo.selector = inspyred.ec.selectors.tournament_selection
	evo.variator = [inspyred.ec.variators.n_point_crossover,inspyred.ec.variators.scramble_mutation]
	evo.replacer = inspyred.ec.replacers.plus_replacement
	evo.terminator = inspyred.ec.terminators.evaluation_termination

	population = evo.evolve(
	 	generator = generator,
	 	evaluator =  evaluator_2,
	 	pop_size = 10000,
	 	num_selected = 2000,
	 	maximize = False,
	 	max_evaluations = 10000,
		max_rotation_amplitude =120,
		max_deplacement_amplitude=100,
		min_number_of_moves = 6,
		max_number_of_moves = 30
	)    
	
	bestListOfCommands=population[0].candidate
	print(bestListOfCommands)
	display(bestListOfCommands)
	
	return 0

if __name__ == "__main__" :
	sys.exit( main() )

