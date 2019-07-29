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
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showCollisionModels')
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('NewProximityIntersection', contactDistance=0.5, alarmDistance=0.8)
        rootNode.createObject('DefaultContactManager', response='default')
        rootNode.createObject('DefaultCollisionGroupManager', name='Group')

        # # rootNode/Floor
        # scale=[10, 1, 10]
        # translation = [100, 0, 100]
        # Floor = rootNode.createChild('Floor')
        # self.Floor = Floor
        # Floor.createObject('MeshObjLoader', name='loader', filename='mesh/floorFlat.obj')
        # Floor.createObject('MeshTopology', src='@loader')
        # Floor.createObject('MechanicalObject', name='mecha', src='@loader', scale3d=scale, translation=translation)
        # Floor.createObject('TTriangleModel', simulated=0, moving=0)
        # Floor.createObject('TLineModel', simulated=0, moving=0)
        # Floor.createObject('TPointModel', simulated=0, moving=0)
        # Floor.createObject('OglModel', name='visu', src='@loader', scale3d=scale, translation=translation)
        
        # rootNode/Cylinder
        scale = [30, 30, 30]
        Cylinder = rootNode.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', rayleighMass='0.1')
        Cylinder.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', iterations='25')
        Cylinder.createObject('CylinderGridTopology', nx='3', ny='3', length='2', radius='0.425', nz='2', axis='0 0 1')
#        Cylinder.createObject('MeshGmshLoader', name='loader', filename='mesh/cylinder.msh')
#        Cylinder.createObject('MeshTopology', src="@loader", name='topo')
        Cylinder.createObject('MechanicalObject', name='Cyl', template='Vec3d')
        Cylinder.createObject('TetrahedronFEMForceField', youngModulus='1e3', poissonRatio='0.4', computeGlobalMatrix='false', method='large')
#        Cylinder.createObject('MeshSpringForceField', tetrasStiffness='1000', tetrasDamping='1.0')
#        Cylinder.createObject('RestShapeSpringsForceField', stiffness='1e3')
        Cylinder.createObject('UniformMass', totalMass='5.0')
        Cylinder.createObject('FixedConstraint', indices='0 1 2 3 4 5 6 7 8')
#        Cylinder.createObject('UncoupledConstraintCorrection')
        Cylinder.createObject('TPointModel')
        Cylinder.createObject('TLineModel')
        Cylinder.createObject('TTriangleModel')

        # Visual Node
        # VisuNode = Cylinder.createChild('Visu_Cyl')
        # VisuNode.createObject('MeshObjLoader', name='visual_loader', filename='mesh/cylinder.obj')
        # VisuNode.createObject('OglModel', name='visual_cyl', src='@visual_loader', color='green')
        # VisuNode.createObject('BarycentricMapping', input='@..', output='@visual_cyl')

        # # Collision Node
        # CollNode = Cylinder.createChild('Coll_Cyl')
        # CollNode.createObject('MeshObjLoader', name='loader', filename='mesh/cylinder.obj')
        # CollNode.createObject('MeshTopology', src='@loader')
        # CollNode.createObject('MechanicalObject', src='@loader')
        # CollNode.createObject('TPointModel')
        # CollNode.createObject('TLineModel')
        # CollNode.createObject('TTriangleModel')
        # CollNode.createObject('BarycentricMapping', input='@..', output='@.')
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
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = SpringEnv(rootNode,commandLineArguments)
    
    return 0
