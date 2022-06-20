# Simple script that simulates a bot moving inside an Arena, following a series of commands
# by Alberto Tonda, 2018 <alberto.tonda@gmail.com>
from sre_parse import State
import sys
import inspyred 
import random
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Circle, Polygon
import numpy as np

import descartes
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
	return out

def evaluator_2(candidates,args):
	out = []
	for candidate in candidates:
		out.append(sg.LineString(candidate).length)
	return out

def evaluator(candidates,args):
    out = []
    for candidate in candidates:
        out.append(args["a"]*sg.LineString([INITIAL_POSITION]+candidate).length+args["b"]*sg.LineString([candidate[-1],OBJECTIVE]).length+penality(candidate,args))
    return out

def penality(candidate,args):
    traj = sg.LineString([INITIAL_POSITION]+candidate+[OBJECTIVE])
    totalPenality = not(traj.intersection(ARENA_LIMITS).is_empty)*1e3
    totalPenality = traj.intersects(traj)*1e6

    for wall in WALLS:
        totalPenality += traj.intersection(wall).length*10
    return totalPenality



def generator(random,args):
    points = []
    accessible=sg.Polygon(ARENA_LIMITS)
    for wall in WALLS: 
        accessible = accessible.difference(wall)

    minx, miny, maxx, maxy = accessible.bounds
    number_of_points = random.randint(args["min_nb_points"],args["max_nb_points"])
    while len(points) < number_of_points:
        pnt = sg.Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        if accessible.contains(pnt):
            points.append(pnt)
    return points

def generator(random,args):
    points = []
    accessible=sg.Polygon(ARENA_LIMITS)
    for wall in WALLS: 
        accessible = accessible.difference(wall)

    minx, miny, maxx, maxy = accessible.bounds
    number_of_points = random.randint(args["min_nb_points"],args["max_nb_points"])
    points.append(sg.Point(random.uniform(minx, maxx), random.uniform(miny, maxy)))
    points.append(sg.Point(random.uniform(minx, maxx), random.uniform(miny, maxy)))
    while len(points) < number_of_points:
        pnt = sg.Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
        line = sg.LineString(points)
        if accessible.contains(pnt):
            points.append(pnt)
    return points

## Affichage des résultats
def display(candidate): 
    figure = plt.figure()
    ax = figure.add_subplot(111)
	
	# plot initial position and objective
    ax.plot(INITIAL_POSITION.x, INITIAL_POSITION.y, 'r^', label="Initial position of the robot")
    ax.plot(OBJECTIVE.x, OBJECTIVE.y, 'gx', label="Position of the objective")
	
	# plot the walls
    for wall in WALLS :
        xs,ys = wall.exterior.xy
        ax.fill(xs,ys,alpha=0.5,fc='b',ec='none')
        
    #Plot de la trajectoire sans collision	
    positions = [INITIAL_POSITION]+candidate+[OBJECTIVE]
    
    lineWithoutCollision=sg.LineString(positions)
    collisionFreeX,collisionFreeY = lineWithoutCollision.xy
    ax.plot(collisionFreeX,collisionFreeY, 'r--', label="Robot path without collision" )
    
    index = 0
    positions = [INITIAL_POSITION]+candidate
    collide = False
    i = 1
    while i < len(positions):
        # Calcul de la nouvelle position à partir de la commande 
        
        traj = sg.LineString([positions[i-1],positions[i]])

        # Vérification de l'intersection entre les limites de l'arène et la trajectoire
        collide = not(traj.intersection(ARENA_LIMITS).is_empty)
        k = 0
        
        # Vérification que l'intersection de la trajectoire et les murs est nulle
        while k < len(WALLS) and not(collide):
            collide = not(traj.intersection(WALLS[k]).is_empty)
            k+=1
        
        # Si pas de collision, on ajoute la position à la liste.
        if not(collide):
            index +=1
        else: 
            break

        i+=1
    
    
    try:
        line=sg.LineString(positions[0:index])
        lx,ly = line.xy
        ax.plot(lx,ly, 'r-', label="Robot" )
    except: 
        print("No direct route")
    
    

    # Limites de l'arène
    xLimits,yLimits=ARENA_LIMITS.xy
    ax.plot(xLimits,yLimits,'b-',label='Arena limits')
    ax.set_title("Movements of the robot inside the arena")
    ax.legend(loc='best')
    plt.show()

## Mutations
################# MAIN
def gaussianMutation(random,candidates,args):
    out = []
    for candidate in candidates:
        out.append(candidate.copy())
        n = random.randint(0,len(candidate)-1)
        out[-1][n]=sg.Point(candidate[n].x+random.gauss(args["mean"],args["sigma"]),candidate[n].y+random.gauss(args["mean"],args["sigma"]))
    return out
def main() :
    listOfCommands = []
    display([INITIAL_POSITION,INITIAL_POSITION])

	# Random generator
    random_generator = random.Random()
    random_generator.seed(42)
	

	#Inspyred 
    evo = inspyred.ec.EvolutionaryComputation(random_generator)
    evo.selector = inspyred.ec.selectors.tournament_selection
    evo.variator = [inspyred.ec.variators.n_point_crossover,gaussianMutation]
    evo.replacer = inspyred.ec.replacers.plus_replacement
    evo.terminator = inspyred.ec.terminators.evaluation_termination
    
    population = evo.evolve(
	 	generator = generator,
	 	evaluator =  evaluator,
	 	pop_size = 3000,
	 	num_selected = 2000,
	 	maximize = False,
        mutation_rate = 0.8,
        crossover_rate = .1,
	 	max_evaluations = 25000,
		min_nb_points = 20,
        max_nb_points = 20,
        mean=0,
        sigma = 5,
        a=0.5,
        b=0.5,
	)
    bestListOfCommands=population[0].candidate 
    line = sg.LineString(bestListOfCommands) 
    print(bestListOfCommands)
    #print(bestListOfCommands.intersection(bestListOfCommands))
    display(bestListOfCommands)
    return 0

if __name__ == "__main__" :
	sys.exit( main() )

