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
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels')# showCollisionModels showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=0.5)
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
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
        
#         # rootNode/Spring
        Spring = rootNode.createChild('Spring')
        self.Spring = Spring
        Spring.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.03', name='odesolver', rayleighMass='1')
        Spring.createObject('CGLinearSolver', threshold='1e-18', tolerance='1e-12', name='linearSolver', iterations='20')

        Spring.createObject('MeshSTLLoader', name='loader', filename='meshes/Spring.STL')
        Spring.createObject('MeshTopology', name='topo', src='@loader')
        Spring.createObject('SparseGridRamificationTopology', n='4 12 3', src='@topo', nbVirtualFinerLevels='3', finestConnectivity='0')
        Spring.createObject('MechanicalObject', name='spring', rotation='0 0 90', translation=[0, -2, 0])
        Spring.createObject('HexahedronFEMForceField', youngModulus='3e3', poissonRatio='0.3', method='large', updateStiffnessMatrix='false')
        Spring.createObject('UniformMass', totalMass='1.0')
#        Spring.createObject('FixedConstraint', indices='93', name='FixedConstraint')
        Spring.createObject('UncoupledConstraintCorrection')

        # rootNode/Spring/VisuSpring
        VisuSpring = Spring.createChild('VisuSpring')
        VisuSpring.createObject('OglModel', name='visu', src='@../loader')
        VisuSpring.createObject('BarycentricMapping', input='@../spring', output='@visu')

        # rootNode/Spring/CollSpring
        CollSpring = Spring.createChild('CollSpring')
        CollSpring.createObject('MechanicalObject', name='coll', src="@../loader")
        CollSpring.createObject('TTriangleModel')
        CollSpring.createObject('TLineModel')
        CollSpring.createObject('TPointModel')
        CollSpring.createObject('BarycentricMapping', input='@..', output='@.')

        # rootNode/Spring1
        rotation = [-90,0,0]
        translation=[45,2,15]
        Spring1 = rootNode.createChild('Spring1')
        self.Spring1 = Spring1
        Spring1.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness=0.03, rayleighMass=1)
        Spring1.createObject('CGLinearSolver', threshold=1e-9, tolerance=1e-9, iterations=20)

        Spring1.createObject('MeshSTLLoader', name='loader', filename='meshes/spring_mcmaster.STL')
        Spring1.createObject('MeshTopology', name='topo', src='@loader')

#        Spring1.createObject('TriangleSetTopologyContainer', name='topo', src='@loader')
#        Spring1.createObject('TriangleSetGeometryAlgorithms', name='algo', template='Vec3d', recomputeTrianglesOrientation=1)
        
        Spring1.createObject('SparseGridRamificationTopology', n=[4, 12, 3], src='@topo', nbVirtualFinerLevels=3, finestConnectivity=0)
        Spring1.createObject('MechanicalObject', name='spring', template='Vec3d', rotation=rotation, translation=translation)
        Spring1.createObject('HexahedronFEMForceField', youngModulus='3e3', poissonRatio='0.3', method='large', updateStiffnessMatrix='false')
#        Spring1.createObject('TriangularFEMForceField', youngModulus=3e5, poissonRatio=0.3, method='large')
#        Spring1.createObject('TriangularBendingSprings', stiffness=6e5, damping=1.0)
               
        Spring1.createObject('UniformMass', totalMass=1.0)
        Spring1.createObject('UncoupledConstraintCorrection')

        # rootNode/Spring1/VisuSpring1
        VisuSpring1 = Spring1.createChild('VisuSpring1')
        VisuSpring1.createObject('OglModel', name='visu', src='@../loader')
        VisuSpring1.createObject('BarycentricMapping', input='@../spring', output='@visu')

        # rootNode/Spring1/CollSpring1
        CollSpring1 = Spring1.createChild('CollSpring1')
        CollSpring1.createObject('MechanicalObject', name='coll', src="@../loader")
        CollSpring1.createObject('TTriangleModel')
        CollSpring1.createObject('TLineModel')
        CollSpring1.createObject('TPointModel')
        CollSpring1.createObject('BarycentricMapping', input='@..', output='@.')

        
        # rootNode/Cylinder
        scale = [0.8, 0.8, 0.8]
        translation = [-30, 100, -65]
        Cylinder = rootNode.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Cylinder.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        Cylinder.createObject('MeshObjLoader', name='loader_cyl', filename='meshes/cylinder_rot.obj')
        Cylinder.createObject('MechanicalObject', name='Cyl', template='Rigid3d', scale3d=scale, translation=translation)
        Cylinder.createObject('UniformMass', totalMass='10000.0')
        Cylinder.createObject('UncoupledConstraintCorrection')

        # # Visual Node
        VisuNode = Cylinder.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual_cyl', src='@../loader_cyl', color='green', scale3d=scale)#, translation=translation)
        VisuNode.createObject('RigidMapping', input='@..', output='@visual_cyl')

        # # Collision Node
        CollNode = Cylinder.createChild('Coll_Cyl')
        CollNode.createObject('MeshTopology', src="@../loader_cyl")
        CollNode.createObject('MechanicalObject', src='@../loader_cyl', scale3d=scale)#, translation=translation)
        CollNode.createObject('TPointModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('RigidMapping')

        # rootNode/Cylinder1
        scale = [0.8, 0.8, 0.8]
        translation = [30, 100, -65]
        Cylinder1 = rootNode.createChild('Cylinder1')
        self.Cylinder1 = Cylinder1
        Cylinder1.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Cylinder1.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        Cylinder1.createObject('MeshObjLoader', name='loader_cyl', filename='meshes/cylinder_rot.obj')
        Cylinder1.createObject('MechanicalObject', name='Cyl', template='Rigid3d', scale3d=scale, translation=translation)
        Cylinder1.createObject('UniformMass', totalMass='10000.0')
        Cylinder1.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNode1 = Cylinder1.createChild('Visu_Cyl')
        VisuNode1.createObject('OglModel', name='visual_cyl', src='@../loader_cyl', color='yellow', scale3d=scale)#, translation=translation)
        VisuNode1.createObject('RigidMapping', input='@..', output='@visual_cyl')

        # Collision Node
        CollNode1 = Cylinder1.createChild('Coll_Cyl')
        CollNode1.createObject('MeshTopology', src="@../loader_cyl")
        CollNode1.createObject('MechanicalObject', src='@../loader_cyl', scale3d=scale)#, translation=translation)
        CollNode1.createObject('TPointModel')
        CollNode1.createObject('TLineModel')
        CollNode1.createObject('TTriangleModel')
        CollNode1.createObject('RigidMapping')
        
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
