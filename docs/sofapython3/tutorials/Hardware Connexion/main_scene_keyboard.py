import Sofa

from KeyboardController import *


name = 'goal'
goal_step = 2 # shift step on key pressed

def EffectorGoal(node, position,name):
    goal = node.addChild(name)
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
    goal.addObject('MechanicalObject', name=name + 'M0', position=position)
    goal.addObject('SphereCollisionModel', radius='0.5')
    # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    goal.addObject('UncoupledConstraintCorrection')
    return goal

def createScene(rootNode):
    goalNode = EffectorGoal(node = rootNode, position = [0.0, 0.0, 0.0],name = name)
    rootNode.addObject(GoalKeyboardController(goal_step = goal_step,child_name = 'goal',name = 'goalM0', RootNode = rootNode))
    rootNode.addObject('VisualStyle', displayFlags="showBehavior showVisual showCollision" )