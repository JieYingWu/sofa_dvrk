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
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython SofaCUDA SofaOpenglVisual')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels')# showCollisionModels showInteractionForceFields showForceFields')
#        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
#        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DefaultContactManager')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.1, alarmDistance=0.5)
        rootNode.createObject('DiscreteIntersection')
#        rootNode.createObject('CollisionGroup')
#        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
#        rootNode.createObject('AnimationLoopParallelScheduler', threadNumber=2)

        # rootNode/Floor
        scale=[10, 1, 10]
        translation = [100, 0, 100]
        Floor = rootNode.createChild('Floor')
        self.Floor = Floor
        Floor.createObject('MeshObjLoader', name='loader', filename='mesh/floorFlat.obj')
        Floor.createObject('MeshTopology', src='@loader')
        Floor.createObject('MechanicalObject', name='mecha', src='@loader', scale3d=scale, translation=translation)
        Floor.createObject('TTriangleModel', simulated=0, moving=0)
        Floor.createObject('TLineModel', simulated=0, moving=0)
        Floor.createObject('TPointModel', simulated=0, moving=0)
        Floor.createObject('OglModel', name='visu', src='@loader', scale3d=scale, translation=translation)
        
        # rootNode/Spring
        Spring = rootNode.createChild('Spring')
        self.Spring = Spring
        Spring.createObject('EulerImplicitSolver', rayleighStiffness='0.03', rayleighMass='1')
        Spring.createObject('CGLinearSolver', threshold='1e-15', tolerance='1e-15', iterations='25')
        Spring.createObject('MeshSTLLoader', name='loader', filename='meshes/steel_extension_spring.stl')
        Spring.createObject('MeshTopology', name='topo', src='@loader')
#        Spring.createObject('SparseGridTopology', n='40 40 40', src='@topo')
        Spring.createObject('MechanicalObject', name='spring', rotation='0 0 90', translation=[0, 25, 0], template='Vec3d')
        Spring.createObject('TriangleFEMForceField', youngModulus='1e5', poissonRatio='0.26')
#        Spring.createObject('TetrahedronFEMForceField', youngModulus='1e9', poissonRatio='0.26')
#        Spring.createObject('MeshShapeSpringsForceField', stiffness='5')
        Spring.createObject('UniformMass', vertexMass='1.0')
#        Spring.createObject('FixedConstraint', indices='93', name='FixedConstraint')
        Spring.createObject('UncoupledConstraintCorrection')

        # rootNode/Spring/VisuSpring
        VisuSpring = Spring.createChild('VisuSpring')
        VisuSpring.createObject('OglModel', name='visu', src='@../loader', template='ExtVec3d', rotation='0 0 90', translation=[0, 25, 0])
        VisuSpring.createObject('BarycentricMapping', input='@..', output='@visu')

        # rootNode/Spring/CollSpring
        CollSpring = Spring.createChild('CollSpring')
        CollSpring.createObject('MechanicalObject', name='coll', src="@../loader", rotation='0 0 90', translation=[0, 25, 0])
        CollSpring.createObject('TTriangleModel')
        CollSpring.createObject('TLineModel')
        CollSpring.createObject('TPointModel')
#        CollSpring.createObject('TetrahedronModel')
        CollSpring.createObject('BarycentricMapping', input='@..', output='@.')
        
        # rootNode/Cylinder
        scale = [0.1, 0.1, 0.1]
        translation = [10, 38, 4]
        rotation = [-90, 0, 90]
        Cylinder = rootNode.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Cylinder.createObject('CGLinearSolver', threshold='1e-15', tolerance='1e-15', name='linearSolver', iterations='25')
        Cylinder.createObject('MeshObjLoader', name='loader_cyl', filename='meshes/cylinder_rot.obj')
        Cylinder.createObject('MeshTopology', src='@loader_cyl')
        Cylinder.createObject('MechanicalObject', name='Cyl', scale3d=scale, translation=translation, rotation=rotation)
#        Cylinder.createObject('UniformMass', totalMass='10000.0')
        Cylinder.createObject('TriangleFEMForceField', youngModulus='1e5', poissonRatio='0.26')
        Cylinder.createObject('TPointModel', simulated=0, moving=0)
        Cylinder.createObject('TLineModel', simulated=0, moving=0)
        Cylinder.createObject('TTriangleModel', simulated=0, moving=0)
        Cylinder.createObject('OglModel', name='visual_cyl', src='@loader_cyl', color='green', scale3d=scale, translation=translation, rotation=rotation)
        Cylinder.createObject('UncoupledConstraintCorrection')

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
    rootNode.findData('dt').value = '0.02'
    rootNode.findData('gravity').value = '0 -10 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = SpringEnv(rootNode,commandLineArguments)
    
    return 0
