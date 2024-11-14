# -*- coding: utf-8 -*-
import Sofa
import Sofa.constants.Key as Key

class PressureController(Sofa.Core.Controller): # A factoriser ailleurs ?
    """
        FR :
        Fonction pour pouvoir modifier les pressions appliqués par le clavier
            INPUT : 
            pas = step, incrément en pression (kPa) à chaque frappe de clavier
            module = variable stiff qui contient toutes les données du robot
            parent = noeud parent des cavités pour s'y connecter

        EN :
        Function to be able to modify the pressures applied by the keyboard
             INPUT:
             pas = step, increment in pressure (kPa) with each keystroke
             module = variable stiff which contains all the data of the robot
             parent = parent node of the cavities to connect them

        Exemple : rootNode.addObject(StiffController(pas=pas,module = stiff,parent = stiff_flop))
    """

    def __init__(self,pas,parent,*args, **kwargs):

            Sofa.Core.Controller.__init__(self,args,kwargs)

            self.pressure = parent.getObject('SurfacePressureConstraint')
            self.flag = 0;
            self.pas = pas
            self.max_pression = 300
            

    def onKeypressedEvent(self,e):
    
            pressureValue = self.pressure.value.value[0]

            if e["key"] == Key.A:
                pressureValue += self.pas
                # print('===========D')
                if pressureValue > self.max_pression:
                    pressureValue= self.max_pression
            if e["key"] == Key.Q:
                pressureValue -= self.pas
                if pressureValue < 0:
                    pressureValue = 0
                        
            self.pressure.value =  [pressureValue]
            print('Actuation pressure : ', pressureValue)   

def getOrAddTheTemplateNode(attachedAsAChildOf=None, attachedTo=None, name=None):
    if attachedTo != None:
        if name != None:
            Sofa.msg_error(attachedTo,
                           "The name parameter cannot be used when a template is attachedTo to an existing node")
            return attachedTo
        if attachedAsAChildOf != None:
            Sofa.msg_error(attachedTo, "Both attachedTo and attachedAsAChildOf are set is not allowed.")
            return attachedTo
        return attachedTo
    return attachedAsAChildOf.addChild(name)

def PneumaticBase(surfaceMeshFileName=None,
                    attachedAsAChildOf=None,
                    attachedTo=None,
                    name="PneumaticCavity",
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1,
                    initialValue=0,
                    valueType="volumeGrowth",
                    points = None,
                    triangles = None,
                    mechanical_object_name = "MechanicalObject" ) :
    """Adds a pneumatic constraint.

    The constraint apply to a parent mesh.

    Args:
        surfaceMeshFileName (string): path to the cavity mesh (the mesh should be a surface mesh, ie only triangles or quads).
        attachedAsAChildOf: default is None
        attachedTo: default is None
        name (string): name of the added node.
        rotation (vec3): default is [0,0,0]
        translation (vec3): default is [0,0,0]
        uniformScale (real): uniform scale, default is 1.
        initialValue (real): value to apply, default is 0.
        valueType (string): type of the parameter value (volumeGrowth or pressure), default is volumeGrowth.

    Structure:
    .. sourcecode:: qml
        Node : {
                name : "PneumaticCavity"
                MeshTopology,
                MechanicalObject,
                SurfacePressureConstraint,
                BarycentricMapping
        }

    Factorised to be both used for direct or inverse control

    """
    if attachedAsAChildOf is None and attachedTo is None:
        Sofa.msg_error(
            "Your PneumaticCavity isn't link/child of any node, please set the argument attachedTo or attachedAsAChildOf")
        return None

    pneumatic = getOrAddTheTemplateNode(attachedAsAChildOf=attachedAsAChildOf,
                                        attachedTo=attachedTo,
                                        name=name)

    if surfaceMeshFileName is None:
        if points is None :
            Sofa.msg_error("No surfaceMeshFileName and no points specified, please specify one")
            return None
        else :
            pneumatic.addObject("TriangleSetTopologyContainer", triangles = triangles,name="MeshLoader",points = points)#, position =Boite_III_K.pointsInROI.value, points = Boite_III_K.indices.value, quad = Boite_III_K.quadIndices.value )

    else :

        # This add a MeshSTLLoader, a component loading the topology of the cavity.
        if surfaceMeshFileName.endswith(".stl"):
            pneumatic.addObject('MeshSTLLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation,
                                translation=translation, scale=uniformScale)
        elif surfaceMeshFileName.endswith(".obj"):
            pneumatic.addObject('MeshOBJLoader', name='MeshLoader', filename=surfaceMeshFileName, rotation=rotation,
                                translation=translation, scale=uniformScale)
        else:
            Sofa.msg_error(
                "Your surfaceMeshFileName extension is not the right one, you have to give a surfacic mesh with .stl or .obj extension")
            return None

    # This adds a MeshTopology, a component holding the topology of the cavity.
    # pneumatic.addObject('MeshTopology', name="topology", filename=surfaceMeshFileName)
    pneumatic.addObject('MeshTopology', name='topology', src='@MeshLoader')

    # This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a cavity actuated with pneumatic, it is a set of positions specifying
    # the points where the pressure is applied.
    pneumatic.addObject('MechanicalObject', src="@topology",name = mechanical_object_name)
    # pneumatic.addObject('MechanicalObject',name = mechanical_object_name)


    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cavity's DoFs and the parents' ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    # pneumatic.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    return pneumatic

def PneumaticCavity(surfaceMeshFileName=None,
                    attachedAsAChildOf=None,
                    attachedTo=None,
                    name="PneumaticCavity",
                    rotation=[0.0, 0.0, 0.0],
                    translation=[0.0, 0.0, 0.0],
                    uniformScale=1,
                    initialValue=0,
                    valueType="volumeGrowth",
                    points = None,
                    triangles = None,
                    mechanical_object_name = "MechanicalObject"):
    """Adds a pneumatic constraint.

    The constraint apply to a parent mesh.

    Args:
        surfaceMeshFileName (string): path to the cavity mesh (the mesh should be a surface mesh, ie only triangles or quads).
        attachedAsAChildOf: default is None
        attachedTo: default is None
        name (string): name of the added node.
        rotation (vec3): default is [0,0,0]
        translation (vec3): default is [0,0,0]
        uniformScale (real): uniform scale, default is 1.
        initialValue (real): value to apply, default is 0.
        valueType (string): type of the parameter value (volumeGrowth or pressure), default is volumeGrowth.

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
    pneumatic = PneumaticBase(surfaceMeshFileName=surfaceMeshFileName,
                    attachedAsAChildOf=attachedAsAChildOf,
                    attachedTo=attachedTo,
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
    pneumatic.addObject('SurfacePressureConstraint',
                        value=initialValue,
                        valueType=valueType)

    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    # between the cavity's DoFs and the parents' ones so that the pressure applied on the cavity wall will be mapped
    # to the volume structure and vice-versa;
    # pneumatic.addObject('BarycentricMapping', name="Mapping", mapForces=False, mapMasses=False)
    return pneumatic


# Exemple doesn't work
def createScene(node):
    from stlib3.scene import MainHeader

    node.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader]  
    node.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping]  
    node.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    node.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Constant') # Needed to use components [MeshTopology]  
    node.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  
    node.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') # Needed to use components [OglSceneFrame]  

    node.addObject('FreeMotionAnimationLoop')
    node.addObject('DefaultVisualManagerLoop')    
    node.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    node.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')

    MainHeader(node, plugins=["SoftRobots", 'SofaPython3'])
    node.addObject('MechanicalObject')

    pneumatic = PneumaticCavity(surfaceMeshFileName="mesh/cube.obj", attachedAsAChildOf=node)
    pneumatic.addObject('TriangleCollisionModel', moving='0', simulated='1') # For visualisation
    pneumatic.addObject('TriangleFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.49,  youngModulus=10000, thickness = 5) # stable youngModulus = 500 / réel ? = 103
    pneumatic.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixMat3x3d")
    pneumatic.addObject('GenericConstraintCorrection')
    pneumatic.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    pneumatic.addObject('RestShapeSpringsForceField', points=[0,1,2,3], angularStiffness=1e5, stiffness=1e5) # pour accrocher la base du robot dans l'espace
    pneumatic.addObject('UniformMass', totalMass=1000, rayleighMass = 0)

    node.addObject(PressureController(pas=10,parent = pneumatic))


