from softrobots.inverse.effectors import positionEffector as pE

def EffectorGoal(attachedTo=None,
    name="Goal",
    position=None,
    template="Vec3",
    translation=[0.0,0.0,0.0],
    rotation=[0.0,0.0,0.0],
    uniformScale=1.0,
    visuScale=0.1,
    SOLVER = True,
    mechanical_object_name = None):

    """Adds a effector goal. Should be used in the context of the resolution of an inverse problem: find the actuation that leads to the given desired position.
    See examples in SoftRobots/docs/tutorials

    Args:
        name (str): Name of the effector goal.

        position: Location of the target position(s) of the effector(s).

        template:

        translation (vec3):   Apply a 3D translation to the object.

        rotation (vec3):   Apply a 3D rotation to the object in Euler angles.

        uniformScale (vec3):   Apply an uniform scaling to the object.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "Goal"
                    EulerImplicit,
                    CGLinearSolver,
                    MechanicalObject
            }

    """
    #  This add a new node in the scene. This node should be appended to the root node.
    goal = attachedTo.addChild(name)

    if mechanical_object_name == None :
        mechanical_object_name = name

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a effector it is a set of positions specifying
    # ghe location of the effector
    if SOLVER :
        goal.addObject('EulerImplicitSolver', firstOrder=True)
        goal.addObject('CGLinearSolver', threshold=1e-5, tolerance=1e-5)
    goal.addObject('MechanicalObject',name=mechanical_object_name, template=template, position=position,
                        rotation=rotation, translation=translation, scale=uniformScale,
                        showObject="1", showObjectScale=visuScale, drawMode="1", showColor="255 255 255 255")

    return goal

def CompleteEffectorGoal(attachedTo=None, # node of the effector goal
    bodyNode = None, # node where the effector goal will be connected to
    name="Goal",
    goal_position=None,
    template="Vec3",
    goal_translation=[0.0,0.0,0.0],
    goal_rotation=[0.0,0.0,0.0],
    uniformScale=1.0,
    visuScale=0.1,
    associated_position=None,
    addSOLVER = True,
    orientationCONTROL = False,
    mechanical_object_name = None,
    node_layer = 2,
    print_flag = True,
    indices = None,
    index = None,
    Collision_Model = True,
    pos_weight = 1,
    orientation_weight = 1):

    """Adds an effector goal and the associated controlled point of the body

    Args:
        name (str): Name of the effector goal.

        position: Location of the target position(s) of the effector(s).

        template:

        translation (vec3):   Apply a 3D translation to the object.

        rotation (vec3):   Apply a 3D rotation to the object in Euler angles.

        uniformScale (vec3):   Apply an uniform scaling to the object.


    Structure:
        .. sourcecode:: qml

            Node : {
                    name : "Goal"
                    EulerImplicit,
                    CGLinearSolver,
                    MechanicalObject
            }

    """
    if mechanical_object_name == None :
        mechanical_object_name = name

    if associated_position == None : associated_position = goal_position

    if orientationCONTROL == True and template == "Vec3":
        Sofa.msg_warning(attachedTo, "Attention, to control the orientation of the effector, it require to use the Rigid3 template, will be replaced in the following" )
        template = "Rigid3"

    goal = EffectorGoal(attachedTo=attachedTo,
    name=name,
    position=goal_position,
    template=template,
    translation=goal_translation,
    rotation=goal_rotation,
    uniformScale=uniformScale,
    visuScale=visuScale,
    SOLVER=addSOLVER,
    mechanical_object_name=mechanical_object_name)

    if addSOLVER :
        goal.addObject('UncoupledConstraintCorrection')

    if Collision_Model :
        goal.addObject('SphereCollisionModel', radius=1) # for visualisation

    effector_goal_path = '@' + node_layer * '../' + name+ '/' + mechanical_object_name + '.position'

    if print_flag :
        print("Effector path :"+ effector_goal_path)

    if orientationCONTROL : # création de deux noeuds distincts pour le contrôle en position et en orientation
        controlledPoints = pE.PositionEffector(attachedTo = bodyNode, template = template, effectorGoal = effector_goal_path, position = associated_position, useDirections = [1, 1, 1, 0, 0, 0], weight = pos_weight,indices=indices, index = index ) 
        controlledPoints_orientation = pE.PositionEffector(attachedTo = bodyNode, template = template, effectorGoal = effector_goal_path, position = associated_position,useDirections = [0, 0, 0, 1, 1, 1], weight = orientation_weight,indices=indices, index = index) 
    else :
        controlledPoints = pE.PositionEffector(attachedTo = bodyNode, template = template, effectorGoal = effector_goal_path, position = associated_position)  # "@../../goal/goal.position"

    return goal
