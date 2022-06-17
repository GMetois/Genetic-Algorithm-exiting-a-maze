from enum import Enum
import random

class MoveType(Enum):
    FORWARD = 1
    ROTATE = 2

class Instruction():
    def __init__(self,moveType:MoveType,amplitude:float):
        if moveType == MoveType.ROTATE:
            amplitude %= amplitude
        self.amplitude = amplitude
        self.moveType = moveType
    
    def applyInstruction(self,point:sg.Point,orientation:float):
        if self.moveType == MoveType.ROTATE:
            orientation += self.amplitude
            return (point,orientation+self.amplitude)
        elif self.moveType == MoveType.FORWARD:
            a = point.x+np.cos(np.radians(orientation))*self.amplitude 
            b = point.y+np.sin(np.radians(orientation))*self.amplitude
            return (sg.Point(a,b),orientation)

    def applyInstructions(instructions, initialPoint:sg.Point, initialAngle:float):
        pointList=[initialPoint]
        angleList=[angleList]
        for instruction in instructions:
            (newPoint,newAngle)=instruction.applyInstruction(pointList[-1],angleList[-1])
            pointList.append(newPoint)
            angleList.append(newAngle)
        return (pointList,angleList)



def evaluator_1(candidates,args):
    out = []
    for candidate in candidates:
        # TODO Implement heuristic
        value = 0
        out.append(value)
    return out


def generator(random,args):
    number_of_dimensions = args["number_of_dimensions"]
    max_number_of_moves = args["max_number_of_moves"]
    max_amplitude = args["max_amplitude"]
    actual_number_of_moves = random.uniform(0,max_number_of_moves)
    out = []
    for x in range(number_of_dimensions):
        individual = []
        for y in range(0,max_number_of_moves):
            moveType = random.choice([MoveType.FORWARD,MoveType.ROTATE])
            amplitude = random.uniform(max_amplitude)
            if moveType == MoveType.ROTATE:
                amplitude %= 360
            individual.append((moveType,amplitude))
        out.append(individual)
    return out