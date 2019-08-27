import sys
import math
import Sofa
import socket
import numpy as np
import geometry_util as geo

class SpringEnv (Sofa.PythonScriptController):
    robot_step = 0
    write_step = 0
    step = 0.05
    axis_scale=100
    time = 0

    # Set so the first position is at centre of the platform
    def __init__(self, node, commandLineArguments) : 
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : "+str(commandLineArguments))        
#        self.robot_pos = np.genfromtxt('../dataset/test/' + 'data_cartesian_processed.csv', delimiter=',')
        self.robot_pos = np.genfromtxt('../dataset/2019-08-14-GelPhantom1/dvrk/' + 'data0_robot_cartesian_processed.csv', delimiter=',')
        self.createGraph(node)
        self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
        
    def output(self):
        return


    def populateNonMoving(self, node, filename, translation=[0, 0, 0], rotation=[0, 0, 0], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('MeshObjLoader', name='loader', filename=filename)
        node.createObject('MeshTopology', src='@loader')
        node.createObject('MechanicalObject', name='mecha', src='@loader', scale3d=scale, translation=translation, rotation=rotation)
        node.createObject('TTriangleModel', simulated=0, moving=0)
        node.createObject('TLineModel', simulated=0, moving=0)
        node.createObject('TPointModel', simulated=0, moving=0)
        node.createObject('OglModel', name='visu', src='@loader', scale3d=scale, translation=translation, rotation=rotation, color=color)
        

    def populateRigid(self, node, filename, position=[0,0,0,0,0,0,1], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        node.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
        if (filename[-4:] == '.obj'):
            node.createObject('MeshObjLoader', name='loader', filename=filename)
        elif (filename[-4:] == '.STL' or filename[-4:] == '.stl'):
            node.createObject('MeshSTLLoader', name='loader', filename=filename)
        node.createObject('MechanicalObject', name='mecha', template='Rigid3d', scale3d=scale, position=position)
        node.createObject('UniformMass', totalMass=mass)#, showAxisSizeFactor=str(self.axis_scale))
        node.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNode = node.createChild('Visu')
        VisuNode.createObject('OglModel', name='visual', src='@../loader', color=color, scale3d=scale)
        VisuNode.createObject('RigidMapping', input='@../mecha', output='@visual')

        # Collision Node
        CollNode = node.createChild('Coll')
        CollNode.createObject('MeshTopology', src="@../loader")
        CollNode.createObject('MechanicalObject', src='@../loader', name='coll', scale3d=scale, template='Vec3d')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TPointModel')
        CollNode.createObject('RigidMapping', input='@../mecha', output='@coll')

        return 0

    def populateVec(self, node, filename, translation=[0, 0, 0], rotation=[0, 0, 0], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        node.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
        if (filename[-4:] == '.obj'):
            node.createObject('MeshObjLoader', name='loader', filename=filename)
            node.createObject('MeshTopology', src='@loader', name='topo')
        elif (filename[-4:] == '.STL' or filename[-4:] == '.stl'):
            node.createObject('MeshSTLLoader', name='loader', filename=filename)
            node.createObject('MeshTopology', src='@loader', name='topo')
        elif (filename[-4:] == '.msh'):
            node.createObject('MeshGmshLoader', name='loader', filename=filename, translation=translation)
            node.createObject('HexahedronSetTopologyContainer', name='container', src='@loader')
            node.createObject('HexahedronSetGeometryAlgorithms', template='Vec3d')
            node.createObject('HexahedronSetTopologyModifier')
            node.createObject('HexahedronSetTopologyAlgorithms')
#        node.createObject('HexahedronModel', src="@loader")
        node.createObject('MechanicalObject', name='mecha', template='Vec3d', scale3d=scale, rotation=rotation)
        node.createObject('HexahedronFEMForceField', youngModulus='1e3', poissonRatio='0.3')
        node.createObject('UniformMass', totalMass=mass)
        node.createObject('UncoupledConstraintCorrection')

        # Visual Node
        #VisuNode = node.createChild('Visu_Cyl')
        #VisuNode.createObject('MeshSTLLoader', name='loader', filename='mesh/simpleBeamHexa.msh')
        #VisuNode.createObject('OglModel', name='visual', src='@loader', color=color, scale3d=scale)
        #VisuNode.createObject('IdentityMapping', input='@../mecha', output='@visual')
        
        # Collision Node
        CollNode = node.createChild('Coll')
        CollNode.createObject('MeshTopology', src="@../loader")
        CollNode.createObject('MechanicalObject', src='@../loader', name='coll', scale3d=scale, template='Vec3d')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TPointModel')
        CollNode.createObject('IdentityMapping', input='@../mecha', output='@coll')

        return 0
    

    def createGraph(self,rootNode):
        self.rootNode = rootNode
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython')# SofaOpenglVisual')# SofaCUDA')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels')# showCollisionModels')# showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)  
        rootNode.createObject('BruteForceDetection')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=0.5)
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')

#        Floor = rootNode.createChild('Floor')
#        self.populateNonMoving(Floor, 'mesh/floor.obj', translation=[0,-18.9,0])
#        self.Floor = Floor

        # rootNode/phantom
        Phantom = rootNode.createChild('Phantom')
        scale=[30, 30, 30]
        translation=[-34.35/30, -17.9/30, -19.65/30]
        self.populateVec(Phantom, 'meshes/SimpleBeamHexa_fine.msh', mass=1e3, color='green', scale=scale, translation=translation)
#        self.populateVec(Phantom, 'meshes/gel_phantom_1.msh', mass=1e3, color='green', translation=translation)
        Phantom.createObject('FixedConstraint', indices=[0,3,12,15,68,17,123,121,21,145,13, 210, 38, 227, 14, 292, 52, 309, 308, 50, 267, 270, 49, 246, 2, 188,35, 164,1, 99, 16, 66, 75, 64, 128, 128, 28, 63, 149, 97, 104, 96, 19, 144, 214, 162, 169, 120, 209, 43, 161, 193, 186, 231, 185, 36, 226, 296, 244, 251, 243, 57, 291, 313, 268, 275])
#        Phantom.createObject('MeshExporter', filename='test/mesh', position='@mecha.position', edges='@container.edges', triangles='@container.triangles', tetras='@container.tetras', exportEveryNumberOfSteps=1, format='gmsh')
        
#        self.populateVec(Phantom, 'meshes/gel_phantom_1.STL', mass=1e3, color='green')
        self.Phantom = Phantom

#        # rootNode/Instrument
        Instrument = rootNode.createChild('Instrument')
        self.populateRigid(Instrument, 'mesh/sphere.obj', mass=1e3, color='yellow')
        self.Instrument = Instrument
        return 0

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.Cylinder
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
        self.f = open("test/position" + str(self.robot_step) + ".txt","w")
        pos = np.array(self.Phantom.getObject('mecha').position)
        self.f.write(str(pos) + '\n')
        self.time += self.rootNode.findData('dt').value
        self.f.close()
        
        return 0

    def onLoaded(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def reset(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        self.robot_step = 0
        self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
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

        if (self.robot_step < self.robot_pos.shape[0]):
            self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
            self.robot_step += 1
        else:
            self.rootNode.getRootContext().animate = False
        return 0

def createScene(rootNode):
    np.set_printoptions(threshold=sys.maxsize)
    rootNode.findData('dt').value = '0.01005308'
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = SpringEnv(rootNode,commandLineArguments)
    
    return 0
