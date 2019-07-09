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

    def populateRigid(self, node, filename, translation=[0, 0, 0], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        node.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        node.createObject('MeshObjLoader', name='loader', filename=filename)
        node.createObject('MechanicalObject', name='mecha', template='Rigid3d', scale3d=scale, translation=translation)
        node.createObject('UniformMass', totalMass=mass)
        node.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNode = node.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual', src='@../loader', color=color, scale3d=scale)
        VisuNode.createObject('RigidMapping', input='@..', output='@visual')

        # Collision Node
        CollNode = node.createChild('Coll_Cyl')
        CollNode.createObject('MeshTopology', src="@../loader")
        CollNode.createObject('MechanicalObject', src='@../loader', scale3d=scale)
        CollNode.createObject('TPointModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('RigidMapping')

        return 0
    
    def populateSpring(self, spring, translation):
        rotation = [-90,0,0]
        index = [99]
        spring.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.03', name='odesolver', rayleighMass='1')
        spring.createObject('CGLinearSolver', threshold='1e-18', tolerance='1e-12', name='linearSolver', iterations='20')
        spring.createObject('MeshSTLLoader', name='loader', filename='meshes/spring_mcmaster.STL')
        spring.createObject('MeshTopology', name='topo', src='@loader')
        spring.createObject('SparseGridRamificationTopology', n='4 12 3', src='@topo', nbVirtualFinerLevels='3', finestConnectivity='0')
        spring.createObject('MechanicalObject', name='spring', rotation=rotation, translation=translation)
        spring.createObject('HexahedronFEMForceField', youngModulus='3e3', poissonRatio='0.3', method='large', updateStiffnessMatrix='false')
        spring.createObject('UniformMass', totalMass=1.0)
        spring.createObject('FixedConstraint', indices=index, name='FixedConstraint')
        spring.createObject('UncoupledConstraintCorrection')

        # rootNode/Spring/VisuSpring
        VisuSpring = spring.createChild('Visuspring')
        VisuSpring.createObject('OglModel', name='visu', src='@../loader')
        VisuSpring.createObject('BarycentricMapping', input='@../spring', output='@visu')

        # rootNode/Spring/CollSpring
        CollSpring = spring.createChild('CollSpring')
        CollSpring.createObject('MechanicalObject', name='coll', src="@../loader")
        CollSpring.createObject('TTriangleModel')
        CollSpring.createObject('TLineModel')
        CollSpring.createObject('TPointModel')
        CollSpring.createObject('BarycentricMapping', input='@..', output='@.')

        return 0
    
    def createGraph(self,rootNode):
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython MultiThreading')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels')# showCollisionModels showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=0.5)
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
        # rootNode.createObject('AnimationLoopParallelScheduler', threadNumber=2)

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
        offset = 70
        translation = [0,2,0]
        Spring = rootNode.createChild('Spring')
        self.populateSpring(Spring, translation)
        self.Spring = Spring

        # rootNode/Spring1
        translation=[offset,2,0]
        Spring1 = rootNode.createChild('Spring1')
        self.populateSpring(Spring1, translation)
        self.Spring1 = Spring1

        # rootNode/Spring2
        translation=[0,2,offset]
        Spring2 = rootNode.createChild('Spring2')
        self.populateSpring(Spring2, translation)
        self.Spring2 = Spring2

        # rootNode/Spring3
        translation=[offset,2,offset]
        Spring3 = rootNode.createChild('Spring3')
        self.populateSpring(Spring3, translation)
        self.Spring3 = Spring3

        # rootNode/Tabletop
        scale = [2,2,2]
        translation = [38, 66, 32]
        Tabletop = rootNode.createChild('Tabletop')
        self.populateRigid(Tabletop, 'mesh/floorFlat.obj', translation=translation, scale=scale, mass=10.0, color='green')
        self.Tabletop = Tabletop
        
        # rootNode/Cylinder
        scale = [0.8, 0.8, 0.8]
        translation = [22, 70, -48]
        Cylinder = rootNode.createChild('Cylinder')
        self.populateRigid(Cylinder, 'meshes/cylinder_rot.obj', translation=translation, scale=scale, mass=1e5, color='blue')
        self.Cylinder = Cylinder
        
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
