# -*- coding: utf-8 -*-
import Sofa

from softrobots.actuators import pneumatic as pb
from softrobots.inverse.effectors import effectorGoal as eG

def PneumaticCavity(surfaceMeshFileName=None, # Identical name to the component used for direct control = may be confusing + input / output not identical
                    attachedTo=None,
                    name="PneumaticCavity",
                    minPressure=None,
                    maxPressure=None,
                    minVolumeGrowth=None,
                    maxVolumeGrowth=None,
                    maxVolumeGrowthVariation=None,
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1,
                    points = None,
                    triangles = None,
                    mechanical_object_name = None,
                    valueType = "volumeGrowth",
                    initialValue = 0,
                    SPC_node_name = 'SurfacePressureConstraint'):

    """Adds a pneumatic actuation. Should be used in the context of the resolution of an inverse problem: find the actuation that leads to a desired deformation.
    See documentation at: https://project.inria.fr/softrobot/documentation/constraint/surface-pressure-actuator/

    The constraint is applied to a parent mesh.

    Args:
        cavityMeshFile (string): path to the cavity mesh (the mesh should be a surfacic mesh, ie only triangles or quads).

        name (string): name of the added node.

        minPressure:

        maxPressure:

        minVolumeGrowth:

        maxVolumeGrowth:This scene is using the SoftRobots.Inverse plugin which does not seem available on your system.


        maxVolumeGrowthVariation:

    Structure:
    .. sourcecode:: qml
        Node : {
                name : "PneumaticCavity"
                MeshTopology,
                MechanicalObject,
                SurfacePressureConstraint,
                BarycentricMapping
        }

    """
    pneumatic = pb.PneumaticBase(surfaceMeshFileName=surfaceMeshFileName,
                    attachedAsAChildOf=attachedTo,
                    name=name,
                    rotation=rotation,
                    translation=translation,
                    uniformScale=uniformScale,
                    initialValue=initialValue,
                    valueType=valueType,
                    points = points,
                    triangles = triangles,
                    mechanical_object_name = mechanical_object_name)

    # Add a SurfacePressureConstraint object with a name.
    # the indices are referring to the MechanicalObject's positions.
    pneumatic.addObject('SurfacePressureActuator')

    if minPressure != None : pneumatic.minPressure = minPressure
    if maxPressure != None : pneumatic.maxPressure = maxPressure
    if minVolumeGrowth != None : pneumatic.minVolumeGrowth = minVolumeGrowth
    if maxVolumeGrowth != None : pneumatic.maxVolumeGrowth = maxVolumeGrowth
    if maxVolumeGrowthVariation != None : pneumatic.maxVolumeGrowthVariation = maxVolumeGrowthVariation

    # This add a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cavity's DoFs and the parents's ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    # pneumatic.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    return pneumatic

def createScene(node):
    from stlib3.scene import MainHeader
    
    node.addObject('RequiredPlugin', name='SoftRobots.Inverse') # Where is SofaValidation ? => Deprecated Error in terminal
    node.addObject('RequiredPlugin', name='SoftRobots')
    node.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader]  
    node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping]  
    node.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
    node.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  
    node.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglSceneFrame]  

    node.addObject('FreeMotionAnimationLoop')
    node.addObject('DefaultVisualManagerLoop')    
    node.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    
    node.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001

    MainHeader(node, plugins=["SoftRobots", 'SofaPython3'])
    node.addObject('MechanicalObject')

    pneumatic = PneumaticCavity(surfaceMeshFileName="mesh/cube.obj", attachedTo=node)
    pneumatic.addObject('TriangleCollisionModel', moving='0', simulated='1') # For visualisation
    pneumatic.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.49,  youngModulus=10000, thickness = 5) # stable youngModulus = 500 / réel ? = 103
    pneumatic.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixMat3x3d")
    pneumatic.addObject('GenericConstraintCorrection')
    pneumatic.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    pneumatic.addObject('RestShapeSpringsForceField', points=[0,1,2,3], angularStiffness=1e5, stiffness=1e5) # pour accrocher la base du robot dans l'espace
    pneumatic.addObject('UniformMass', totalMass=1000, rayleighMass = 0)

    goal = eG.CompleteEffectorGoal(attachedTo=node, bodyNode = pneumatic, goal_position = [0,0,1], associated_position = [0,0,1], name = 'goal')

