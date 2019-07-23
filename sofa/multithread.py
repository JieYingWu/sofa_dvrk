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
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython MultiThreading')
        rootNode.createObject('AnimationLoopParallelScheduler', threadNumber=2)

        # rootNode/FloorScene
        FloorScene = rootNode.createChild('FloorScene')
        FloorScene.createObject('VisualStyle', displayFlags='showBehaviorModels')
        FloorScene.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        FloorScene.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        FloorScene.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        FloorScene.createObject('BruteForceDetection', name='N2')
        FloorScene.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=0.5)
        FloorScene.createObject('DiscreteIntersection')
        FloorScene.createObject('DefaultContactManager', name='Response', response='FrictionContact')

        # rootNode/FloorScene/Floor
        scale=[10, 1, 10]
        translation = [100, 0, 100]
        Floor = FloorScene.createChild('Floor')
        self.Floor = Floor
        Floor.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Floor.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        Floor.createObject('MeshObjLoader', name='loader', filename='mesh/floorFlat.obj')
        Floor.createObject('MeshTopology', src='@loader')
        Floor.createObject('MechanicalObject', name='mecha', template='Rigid3d', src='@loader', scale3d=scale, translation=translation)
        Floor.createObject('FixedConstraint', indices='0 1 2 3')
        Floor.createObject('OglModel', name='visu', src='@loader', scale3d=scale, translation=translation)

        CollFloor = Floor.createChild('Coll')
        CollFloor.createObject('MeshTopology', src='@../loader')
        CollFloor.createObject('MechanicalObject', src='@../loader', name='coll', scale3d=scale, translation=translation)
        CollFloor.createObject('TTriangleModel')
        CollFloor.createObject('TLineModel')
        CollFloor.createObject('TPointModel')
        CollFloor.createObject('RigidMapping', input='@..', output='@coll')

        # rootNode/CylinderScene
        CylinderScene = rootNode.createChild('CylinderScene')
        CylinderScene.createObject('VisualStyle', displayFlags='showBehaviorModels')
        CylinderScene.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        CylinderScene.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        CylinderScene.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        CylinderScene.createObject('BruteForceDetection', name='N2')
        CylinderScene.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=0.5)
        CylinderScene.createObject('DiscreteIntersection')
        CylinderScene.createObject('DefaultContactManager', name='Response', response='FrictionContact')  
        # rootNode/Cylinder
        scale = [0.8, 0.8, 0.8]
        translation = [-30, 100, -65]
        Cylinder = CylinderScene.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Cylinder.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        Cylinder.createObject('MeshObjLoader', name='loader_cyl', filename='mesh/cylinder.obj')
        Cylinder.createObject('MechanicalObject', name='Cyl', template='Rigid3d', scale3d=scale, translation=translation)
        Cylinder.createObject('UniformMass', totalMass='10000.0')
        Cylinder.createObject('UncoupledConstraintCorrection')

        # # Visual Node
        VisuNode = Cylinder.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual_cyl', src='@../loader_cyl', color='green', scale3d=scale)
        VisuNode.createObject('RigidMapping', input='@..', output='@visual_cyl')

        # # Collision Node
        CollNode = Cylinder.createChild('Coll_Cyl')
        CollNode.createObject('MeshTopology', src="@../loader_cyl")
        CollNode.createObject('MechanicalObject', src='@../loader_cyl', scale3d=scale)
        CollNode.createObject('TPointModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('RigidMapping')

        rootNode.createObject('DataExchange', name='exchangeData1', template='vector<float>', from='@FloorScene/Floor/mecha.position', to='@CylinderScene/Cylinder/Cyl.position')
        
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
        # pos = np.array(self.Cylinder.getObject('Cyl').position)
        # print('pos is', pos)
        # if c== "Q":
        #     newPos = pos + np.array([1, 0, 0, 0, 0, 0, 0])
        #     newPos = geo.arrToStr(newPos)
        #     print(newPos)
        #     self.Cylinder.getObject('Cyl').position = newPos
        # if c == "W":
        #     newPos = pos + np.array([-1, 0, 0, 0, 0, 0, 0])
        #     newPos = geo.arrToStr(newPos)
        #     self.Cylinder.getObject('Cyl').position = newPos
        # if c == "A":
        #     newPos = pos + np.array([0, 1, 0, 0, 0, 0, 0])
        #     newPos = geo.arrToStr(newPos)
        #     self.Cylinder.getObject('Cyl').position = newPos
        # if c == "D":
        #     newPos = pos + np.array([0, -1, 0, 0, 0, 0, 0])
        #     newPos = geo.arrToStr(newPos)
        #     self.Cylinder.getObject('Cyl').position = newPos
        # if c == "Z":
        #     newPos = pos + np.array([0, 0, 1, 0, 0, 0, 0])
        #     newPos = geo.arrToStr(newPos)
        #     self.Cylinder.getObject('Cyl').position = newPos
        # if c == "X":
        #     newPos = pos + np.array([0, 0, -1, 0, 0, 0, 0])
        #     newPos = geo.arrToStr(newPos)
        #     self.Cylinder.getObject('Cyl').position = newPos
        # if c == "T":
        #     delta_q = geo.eulerToQuaternion([0, 0, 0, 0.1, 0, 0])
        #     print(delta_q)
        #     newPos = geo.q_mult(pos[0], delta_q)
        #     print(newPos)
        #     self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        # if c == "Y":
        #     delta_q = geo.eulerToQuaternion([0, 0, 0, -0.1, 0, 0])
        #     newPos = geo.q_mult(pos[0], delta_q)
        #     self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        # if c == "G":
        #     delta_q = geo.eulerToQuaternion([0, 0, 0, 0, 0.1, 0])
        #     newPos = geo.q_mult(pos[0], delta_q)
        #     self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        # if c == "H":
        #     delta_q = geo.eulerToQuaternion([0, 0, 0, 0, -0.1, 0])
        #     newPos = geo.q_mult(pos[0], delta_q)
        #     self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        # if c == "V":
        #     delta_q = geo.eulerToQuaternion([0, 0, 0, 0, 0, 0.1])
        #     newPos = geo.q_mult(pos[0], delta_q)
        #     self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        # if c == "B":
        #     delta_q = geo.eulerToQuaternion([0, 0, 0, 0, 0, -0.1])
        #     newPos = geo.q_mult(pos[0], delta_q)
        #     self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        # if c == "E":
        #     self.Cylinder.getObject('Cyl').velocity = '0 0 0 0 0 0'


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
    rootNode.findData('gravity').value = '0 -50 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = SpringEnv(rootNode,commandLineArguments)
    
    return 0
