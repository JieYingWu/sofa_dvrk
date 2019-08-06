import sys
import math
import Sofa
import socket
import numpy as np
import geometry_util as geo

class SpringEnv (Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments) : 
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : "+str(commandLineArguments))
        self.createGraph(node)


    def output(self):
        return

    def createGraph(self,rootNode):
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython SofaOpenglVisual')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showCollisionModels')
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('NewProximityIntersection', contactDistance=0.5, alarmDistance=0.8)
        rootNode.createObject('DefaultContactManager', response='default')
        
        # rootNode/Spring
        Spring = rootNode.createChild('Spring')
        self.Spring = Spring
        Spring.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', rayleighMass='0.1')
        Spring.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', iterations='25')
        Spring.createObject('CylinderGridTopology', nx='3', ny='3', length='2', radius='0.425', nz='2', axis='0 -1 0', name='topo')
        Spring.createObject('MechanicalObject', name='Cyl', template='Vec3d')
        Spring.createObject('TetrahedronFEMForceField', youngModulus='1e3', poissonRatio='0.4', computeGlobalMatrix='false', method='large')
#        Spring.createObject('MeshSpringForceField', tetrasStiffness='1000', tetrasDamping='1.0')
#        Spring.createObject('RestShapeSpringsForceField', stiffness='1e3')
        Spring.createObject('UniformMass', totalMass='5.0', showAxisSizeFactor=0.001)
        Spring.createObject('FixedConstraint', indices='0 1 2 3 4 5 6 7 8')
        Spring.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNode = Spring.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual_cyl', src='@../topo', color='green')
        VisuNode.createObject('BarycentricMapping', input='@..', output='@visual_cyl')

        # Collision Node
        CollNode = Spring.createChild('Coll_Cyl')
        CollNode.createObject('MechanicalObject', src='@../topo')
        CollNode.createObject('PointCollisionModel')
        CollNode.createObject('LineCollisionModel')
        CollNode.createObject('TriangleCollisionModel')
        CollNode.createObject('BarycentricMapping', input='@..', output='@.')

        # rootNode/Cylinder
        translation = '0 -3 0'
        Cylinder = rootNode.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', rayleighMass='0.1')
        Cylinder.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', iterations='25')
        Cylinder.createObject('CylinderGridTopology', nx='3', ny='3', length='2', radius='0.425', nz='2', axis='0 -1 0', name='topo')
        Cylinder.createObject('MechanicalObject', name='Cyl', template='Vec3d', translation=translation)
        Cylinder.createObject('TetrahedronFEMForceField', youngModulus='1e3', poissonRatio='0.4', computeGlobalMatrix='false', method='large')
        Cylinder.createObject('UniformMass', totalMass='5.0', showAxisSizeFactor=0.001)
        Cylinder.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNode = Cylinder.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual_cyl', src='@../topo', color='green', translation=translation)
        VisuNode.createObject('BarycentricMapping', input='@..', output='@visual_cyl')

        # Collision Node
        CollNode = Cylinder.createChild('Coll_Cyl')
        CollNode.createObject('MechanicalObject', src='@../topo', translation=translation)
        CollNode.createObject('PointCollisionModel')
        CollNode.createObject('LineCollisionModel')
        CollNode.createObject('TriangleCollisionModel')
        CollNode.createObject('BarycentricMapping', input='@..', output='@.')


        rootNode.createObject('BoxStiffSpringForceField', template='Vec3d', stiffness=1e5, object1='@Spring', object2='@Cylinder', box_object1='-0.5 -1 -0.5 0.5 -2.2 0.5', box_object2='-0.5 -3.5 -0.5 0.5 -2.8 0.5', forceOldBehavior='false')
        return 0

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def onKeyReleased(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You released a"
        return 0

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    # Note: Hold control when key is pressed
    def onKeyPressed(self, c):
        ## usage e.g.
        # print(c, 'has been pressed')
        return 0

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        ## usage e.g.
        #if isPressed : 
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0

    def storeResetState(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        # self.conn.close()
        return 0

    def onGUIEvent(self, strControlID,valueName,strValue):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print 'onGUIEvent'
        return 0

    def onEndAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def onScriptEvent(self, senderNode, eventName,data):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print 'onScriptEvent'
        return 0

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed :
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

def createScene(rootNode):
    rootNode.findData('dt').value = '0.001'
    rootNode.findData('gravity').value = '0 -10 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = SpringEnv(rootNode,commandLineArguments)
    
    return 0
