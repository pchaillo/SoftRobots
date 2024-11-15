from softrobots.inverse.effectors import positionEffector as pE

def EffectorGoal(attachedTo=None,
    name="Goal",
    position=None,
    template="Vec3",
    translation=[0.0,0.0,0.0],
    rotation=[0.0,0.0,0.0],
    uniformScale=1.0,
    visuScale=0.1,
    ):

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

    # This add a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a effector it is a set of positions specifying
    # ghe location of the effector
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', threshold=1e-5, tolerance=1e-5)
    goal.addObject('MechanicalObject',name=name, template=template, position=position,
                        rotation=rotation, translation=translation, scale=uniformScale,
                        showObject="1", showObjectScale=visuScale, drawMode="1", showColor="255 255 255 255")

    return goal

def CompleteEffectorGoal(attachedTo=None,
    bodyNode = None,
    name="Goal",
    goal_position=None,
    template="Vec3",
    goal_translation=[0.0,0.0,0.0],
    goal_rotation=[0.0,0.0,0.0],
    uniformScale=1.0,
    visuScale=0.1,
    associated_position=None
    ):

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
    goal = EffectorGoal(attachedTo=attachedTo,
    name=name,
    position=goal_position,
    template=template,
    translation=goal_translation,
    rotation=goal_rotation,
    uniformScale=uniformScale,
    visuScale=visuScale)

    goal.addObject('UncoupledConstraintCorrection')
    goal.addObject('SphereCollisionModel', radius=1) # for visualisation

    controlledPoints = pE.PositionEffector(attachedTo = bodyNode, template = "Vec3", effectorGoal = "@../../goal/goal.position", position = associated_position)

    # controlledPoints = bodyNode.addChild('controlledPoints')
    # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Vec3",position=associated_position) 
    # controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../goal/goal.position") # Lien entre les deux dépend de la position des noeuds = pas parfait à reprendre
    # controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    return goal
