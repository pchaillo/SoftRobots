
import Sofa.Core
import Sofa.constants.Key as Key
from spicy import *

class GoalKeyboardController(Sofa.Core.Controller):
    # pour controller la position de l'effecteur désiré avec le clavier
    # Controle le point avec décalage
        def __init__(self,goal_step,child_name,name,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]
            self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
            self.position = self.stiffNode.getObject(name)
            self.step = goal_step

        def onKeypressedEvent(self,e):

            d = copy(self.position.position.value)
            if e["key"] == Key.D:
                d[0][0] += self.step  
            if e["key"] == Key.C:
                d[0][0] -= self.step  

            if e["key"] == Key.F:
                d[0][1] += self.step  
            if e["key"] == Key.V:
                d[0][1] -= self.step  

            if e["key"] == Key.G:
                d[0][2] += self.step  
            if e["key"] == Key.B:
                d[0][2] -= self.step 

            self.position.position = [d[0]]