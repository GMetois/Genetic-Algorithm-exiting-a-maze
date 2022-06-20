# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>
import sys
import inspyred 
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

import shapely.geometry as sg

from enum import Enum

## CONSTANTES
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
		i = 0
		collide = False
		while i < len(instructions) and not(collide):
			# Calcul de la nouvelle position à partir de la commande
			newState=instructions[i].applyInstruction(positions[-1],angles[-1])
			
			traj = sg.LineString([newState[0],positions[-1]])

			# Vérification de l'intersection entre les limites de l'arène et la trajectoire
			collide = not(traj.intersection(ARENA_LIMITS).is_empty)
			k = 0
			
			# Vérification que l'intersection de la trajectoire et les murs est nulle
			while k < len(WALLS) and not(collide):
				collide = not(traj.intersection(WALLS[k]).is_empty)
				k+=1
			
			# Si pas de collision, on ajoute la position à la liste.
			if not(collide):
				positions.append(newState[0])
				angles.append(newState[1])
			i+=1
		return (positions, angles)

	def applyInstructionsBis(instructions,initialPoint:sg.Point,initialAngle:sg.Point):
		positions = [initialPoint]
		angles = [initialAngle]
		index = 0
		collide = False
		for instruction in instructions:
			# Calcul de la nouvelle position à partir de la commande
			newState=instruction.applyInstruction(positions[-1],angles[-1])
			positions.append(newState[0])
			angles.append(newState[1])


			traj = sg.LineString([positions[-2],positions[-1]])

			# Vérification de l'intersection entre les limites de l'arène et la trajectoire
			if not(traj.intersection(ARENA_LIMITS).is_empty):
				collide = True
			k = 0
			
			# Vérification que l'intersection de la trajectoire et les murs est nulle
			while k < len(WALLS) and not(collide):
				if not(traj.intersection(WALLS[k]).is_empty):
					collide = True
				k+=1
			
			# Si pas de collision, on ajoute la position à la liste.
			if not(collide):
				index+=1

		return (positions[0:index], angles[0:index],positions,angles)


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
	return (1+coeff)*ligne_droite.length

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

def evaluator_3(candidates,args):
	out = []
	for candidate in candidates:
		positionsWCollisions,anglesWCollisions,positions,angles = Instruction.applyInstructionsBis(candidate,INITIAL_POSITION,INITIAL_ANGLE)
		out.append((1+10000*(len(positionsWCollisions)-len(positions)))*sum([heuristic_2(point) for point in positionsWCollisions]))
	return out

def evaluator_4(candidates,args):
    out = []
    for candidate in candidates:
        positionsWCollisions,anglesWCollisions,positions,angles = Instruction.applyInstructionsBis(candidate,INITIAL_POSITION,INITIAL_ANGLE)
        totalLine = sg.LineString(positionsWCollisions+[INITIAL_POSITION])
        lineUntilHit = sg.LineString(positions)
        totalInt = totalLine.length-lineUntilHit.length
        minimalDistance = 1200
        out.append(args["weight_proximity_wall"]*(1-minimalDistance/1200)+args["weight_dist_inter"]*totalInt+args["weight_dist_traj"]*lineUntilHit.length+args["weight_dist_objective"]*sg.LineString([positions[-1],OBJECTIVE]).length+args["weight_out_arena"]*int(not(totalLine.intersects(ARENA_LIMITS))))
    return out

def evaluator_5(candidates,args):
	out = []
	for candidate in candidates:
		positionsWCollisions,anglesWCollisions,positions,angles = Instruction.applyInstructionsBis(candidate,INITIAL_POSITION,INITIAL_ANGLE)
		totalLine = sg.LineString(positions)
		lineUntilHit = sg.LineString([INITIAL_POSITION]+positionsWCollisions)
		if totalLine.length == 0:
			out.append(1e12)
		else:
			totalInt = args["weight_dist_inter"]*(1-lineUntilHit.length/totalLine.length)
			dist_traj = args["weight_dist_traj"]*lineUntilHit.length
			dist_objective = args["weight_dist_objective"]*(sg.LineString([positions[-1],OBJECTIVE]).length)
			minimalDistance = 1200
			# print("Eval : ")
			# print(totalInt)
			# print(dist_traj)
			# print(dist_objective)
			# print(args["weight_out_arena"]*int(totalLine.intersects(ARENA_LIMITS)))
			# print((1+totalInt+dist_traj+dist_objective+args["weight_out_arena"]*int(totalLine.intersects(ARENA_LIMITS))))
			# print()
			#print(totalInt)
			#out.append((1+args["weight_dist_inter"]*totalInt+args["weight_dist_traj"]*lineUntilHit.length+args["weight_dist_objective"]*(sg.LineString([positions[-1],OBJECTIVE]).length)/totalLine.length+args["weight_out_arena"]*int(totalLine.intersects(ARENA_LIMITS)))*lineUntilHit.length)
			out.append((1+totalInt+dist_traj+dist_objective+args["weight_out_arena"]*int(totalLine.intersects(ARENA_LIMITS)))*totalLine.length)



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
			amplitude = random.randint(1,args["max_deplacement_amplitude"])
		elif moveType == MoveType.ROTATE:
			amplitude = random.randint(0,args["max_rotation_amplitude"])*random.choice([-1,1])
		
		individual.append(Instruction(moveType,amplitude))
	return individual

def generator_2(random,args):
	actual_number_of_moves = random.randint(args["min_number_of_moves"],args["max_number_of_moves"])
	individual = []
	for y in range(actual_number_of_moves):
		if random.random() < 0.50:
			moveType = MoveType.FORWARD
		else: 
			moveType = MoveType.ROTATE
		#moveType = random.choice([MoveType.FORWARD,MoveType.FORWARD,MoveType.FORWARD,MoveType.ROTATE])
		# if individual != []:
		# 	if individual[-1].moveType == moveType.ROTATE:
		# 		moveType = MoveType.FORWARD
		
		if moveType == MoveType.FORWARD:
			#amplitude = random.gammavariate(4,2)
			amplitude =  args["max_deplacement_amplitude"]
		elif moveType == MoveType.ROTATE:
			#amplitude = random.gauss(0,args["max_rotation_amplitude"])
			amplitude = random.gauss(0,args["max_rotation_amplitude"])
		individual.append(Instruction(moveType,amplitude))
	return individual

## Affichage des résultats
def display(instructions): 
    figure = plt.figure()
    ax = figure.add_subplot(111)
    # plot initial position and objective
    ax.plot(INITIAL_POSITION.x, INITIAL_POSITION.y, 'r^', label="Initial position of the robot")
    ax.plot(OBJECTIVE.x, OBJECTIVE.y, 'gx', label="Position of the objective")
    for wall in WALLS :
		# ax.add_patch(patches.Rectangle( (wall["x"], wall["y"]), wall["width"], wall["height"] ))
        xs,ys = wall.exterior.xy
        ax.fill(xs,ys,alpha=0.5,fc='b',ec='none')
	# plot a series of lines describing the movement of the robot in the arena
    pointsWithoutCollision,angleWithoutCollision = Instruction.applyInstructions(instructions,INITIAL_POSITION,INITIAL_ANGLE)
    positions,angles = Instruction.applyInstructionsUntilHit(instructions,INITIAL_POSITION,INITIAL_ANGLE)
    #Plot de la trajectoire sans collision
    lineWithoutCollision=sg.LineString(pointsWithoutCollision)
    collisionFreeX,collisionFreeY = lineWithoutCollision.xy
    ax.plot(collisionFreeX,collisionFreeY, 'r--', label="Robot path without collision" )

	#Plot de la véritable trajectoire (avec collision)
    line = sg.LineString(positions)
    lx,ly = line.xy
    ax.plot(lx,ly, 'r-', label="Robot path" )
	
	# Limites de l'arène 
    xLimits,yLimits=ARENA_LIMITS.xy
    ax.plot(xLimits,yLimits,'b-',label='Arena limits')
    ax.set_title("Movements of the robot inside the arena")
    ax.legend(loc='best')
    plt.show()

## Mutations
def changeAmpl(random,candidates,args):
	out = []
	for candidate in candidates:
		out.append(candidate.copy())
		if random.random() <=args["p_change_move"]: 
			k = random.randint(0,len(candidate)-1)
			if candidate[k].moveType == MoveType.ROTATE:
				out[-1][k].amplitude = random.gauss(0,args["max_rotation_amplitude"])
			elif candidate[k].moveType == MoveType.FORWARD:
				out[-1][k].amplitude = args["max_deplacement_amplitude"]
	return out

def moveTypeMutation(random,candidates,args):
	out = []
	for candidate in candidates:
		out.append(candidate.copy())
		if random.random() <=args["p_change_move"]: 
			k = random.randint(0,len(candidate)-1)
			if candidate[k].moveType == MoveType.ROTATE:
				out[-1][k].moveType = MoveType.FORWARD
				out[-1][k].amplitude = abs(candidate[k].amplitude)/180*args["max_deplacement_amplitude"]
			elif candidate[k].moveType == MoveType.FORWARD:
				out[-1][k].moveType = MoveType.ROTATE
				out[-1][k].amplitude = abs(candidate[k].amplitude)/args["max_deplacement_amplitude"]*180*random.choice([-1,1])
	return out
################# MAIN
def main() :
	listOfCommands = []

	# Random generator
	random_generator = random.Random()
	random_generator.seed(42)
	

	#Inspyred 
	evo = inspyred.ec.EvolutionaryComputation(random_generator)
	
	evo.selector = inspyred.ec.selectors.tournament_selection
	evo.variator = [inspyred.ec.variators.n_point_crossover,moveTypeMutation]
	evo.replacer = inspyred.ec.replacers.plus_replacement
	evo.terminator = inspyred.ec.terminators.evaluation_termination
	
	population = evo.evolve(
	 	generator = generator_2,
	 	evaluator =  evaluator_5,
	 	pop_size = 500,
	 	num_selected = 2000,
	 	maximize = False,
		mutation_rate = 0.9,
		crossover_rate = 0.2,
	 	max_evaluations = 40000,
		max_rotation_amplitude =10,
		max_deplacement_amplitude=5,
		min_number_of_moves = 150,
		max_number_of_moves = 150,
        weight_dist_objective = 1,
        weight_dist_traj = 0,
        weight_dist_inter = 100,
		weight_out_arena = 100,
		weight_proximity_wall = 0
	)
    # Affichage des résultats
	indiv = population[0]
	bestListOfCommands=indiv.candidate
	print(bestListOfCommands)
	print(indiv.fitness)
	#print(evaluator_4([bestListOfCommands],None))
	display(bestListOfCommands)

	return 0


if __name__ == "__main__" :
	sys.exit( main() )

