import sys
import Sofa
import socket
import numpy as np
import geometry_util as geo

class RL_env(Sofa.PythonScriptController):

    def __init__(self, node, commandLineArguments):
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : " + str(commandLineArguments))
        self.createGraph(node)


    def createGraph(self, rootNode):

        gravity = '0 0 0'
        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaOpenglVisual SofaPython')
        rootNode.createObject('VisualStyle', displayFlags='showForceFields') # showCollisionModels')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        rootNode.createObject('DefaultPipeline', draw='0', depth='5', verbose='1')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('MinProximityIntersection', contactDistance='1', alarmDistance='2', name='Proximity')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')

        # rootNode/Cylinder
        size = 0.1
        scale = 1
        shift = 10
        Cylinder = rootNode.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('EulerImplicitSolver', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Cylinder.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        Cylinder.createObject('CylinderGridTopology', nx='5', ny='5', length='25', radius='3', nz='3', axis='1 0 0', name='loader')
#        Cylinder.createObject('MeshTopology', src='@loader')
        Cylinder.createObject('MechanicalObject', name='Cyl', template='Vec3d', src='@loader', dx=shift)
        Cylinder.createObject('TriangleFEMForceField', youngModulus='3e6', poissonRatio='0')
        Cylinder.createObject('UniformMass', totalMass='1')
        Cylinder.createObject('UncoupledConstraintCorrection')
        
        # Visual Node
        VisuNode = Cylinder.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual_cyl', src='@../loader', color='green', dx=shift)
        VisuNode.createObject('BarycentricMapping', input='@../Cyl', output='@visual_cyl')

        # Collision Node
        CollNode = Cylinder.createChild('Coll_Cyl')
        CollNode.createObject('MechanicalObject', src = '@../loader', dx=shift)
        CollNode.createObject('TriangleCollisionModel')
        CollNode.createObject('LineCollisionModel')
        CollNode.createObject('PointCollisionModel')
        CollNode.createObject('BarycentricMapping')
        
        # rootNode/Cube
        Cube = rootNode.createChild('Cube')
        self.Cube = Cube
        Cube.createObject('EulerImplicitSolver', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Cube.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        Cube.createObject('MeshObjLoader', name='loader', filename='mesh/smCube27.obj')
        Cube.createObject('MeshTopology', src='@loader')
        Cube.createObject('MechanicalObject', name='Cube', template='Vec3d', src='@loader')
        Cube.createObject('TriangleFEMForceField', youngModulus='3e6', poissonRatio='0')
        Cube.createObject('UniformMass', totalMass='5')
        Cube.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNodeCube = Cube.createChild('Visu_Cube')
        VisuNodeCube.createObject('OglModel', name='visual_cube', src='@../loader', color='blue')
        VisuNodeCube.createObject('BarycentricMapping', input='@../Cube', output='@visual_cube')

        # Collision Node
        CollNodeCube = Cube.createChild('Coll_Cube')
        CollNodeCube.createObject('MechanicalObject', template='Vec3d', src='@../loader')
        CollNodeCube.createObject('TriangleCollisionModel')
        CollNodeCube.createObject('LineCollisionModel')
        CollNodeCube.createObject('PointCollisionModel')
        CollNodeCube.createObject('BarycentricMapping')

        return 0

    def onMouseButtonLeft(self, mouseX, mouseY, isPressed):
        ## usage e.g.
        # if isPressed :
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def onKeyReleased(self, c):
        ## usage e.g.
        # if c=="A" :
        #    print "You released a"
        return 0

    def initGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    # Note: Hold control when key is pressed
    def onKeyPressed(self, c):
        ## usage e.g.
        print(c, 'has been pressed')
        pos = np.array(self.Cylinder.getObject('Cyl').position)
        print('pos is', pos)
        if c == "Q":
            newPos = pos + np.array([1, 0, 0, 0, 0, 0, 0])
            newPos = geo.arrToStr(newPos)
            print(newPos)
            self.Cylinder.getObject('Cyl').position = newPos
        if c == "W":
            newPos = pos + np.array([-1, 0, 0, 0, 0, 0, 0])
            newPos = geo.arrToStr(newPos)
            self.Cylinder.getObject('Cyl').position = newPos
        if c == "A":
            newPos = pos + np.array([0, 1, 0, 0, 0, 0, 0])
            newPos = geo.arrToStr(newPos)
            self.Cylinder.getObject('Cyl').position = newPos
        if c == "D":
            newPos = pos + np.array([0, -1, 0, 0, 0, 0, 0])
            newPos = geo.arrToStr(newPos)
            self.Cylinder.getObject('Cyl').position = newPos
        if c == "Z":
            newPos = pos + np.array([0, 0, 1, 0, 0, 0, 0])
            newPos = geo.arrToStr(newPos)
            self.Cylinder.getObject('Cyl').position = newPos
        if c == "X":
            newPos = pos + np.array([0, 0, -1, 0, 0, 0, 0])
            newPos = geo.arrToStr(newPos)
            self.Cylinder.getObject('Cyl').position = newPos
        if c == "T":
            delta_q = geo.eulerToQuaternion([0, 0, 0, 0.1, 0, 0])
            print(delta_q)
            newPos = geo.q_mult(pos[0], delta_q)
            print(newPos)
            self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        if c == "Y":
            delta_q = geo.eulerToQuaternion([0, 0, 0, -0.1, 0, 0])
            newPos = geo.q_mult(pos[0], delta_q)
            self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        if c == "G":
            delta_q = geo.eulerToQuaternion([0, 0, 0, 0, 0.1, 0])
            newPos = geo.q_mult(pos[0], delta_q)
            self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        if c == "H":
            delta_q = geo.eulerToQuaternion([0, 0, 0, 0, -0.1, 0])
            newPos = geo.q_mult(pos[0], delta_q)
            self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        if c == "V":
            delta_q = geo.eulerToQuaternion([0, 0, 0, 0, 0, 0.1])
            newPos = geo.q_mult(pos[0], delta_q)
            self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        if c == "B":
            delta_q = geo.eulerToQuaternion([0, 0, 0, 0, 0, -0.1])
            newPos = geo.q_mult(pos[0], delta_q)
            self.Cylinder.getObject('Cyl').position = geo.arrToStr(newPos)
        if c == "E":
            self.Cylinder.getObject('Cyl').velocity = '0 0 0 0 0 0'

        return 0

    def onMouseWheel(self, mouseX, mouseY, wheelDelta):
        ## usage e.g.
        # if isPressed :
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0

    def storeResetState(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def cleanup(self):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        # self.conn.close()
        return 0

    def onGUIEvent(self, strControlID, valueName, strValue):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print
        'onGUIEvent'
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

    def onMouseButtonMiddle(self, mouseX, mouseY, isPressed):
        ## usage e.g.
        # if isPressed :
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def bwdInitGraph(self, node):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0

    def onScriptEvent(self, senderNode, eventName, data):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        print
        'onScriptEvent'
        return 0

    def onMouseButtonRight(self, mouseX, mouseY, isPressed):
        ## usage e.g.
        # if isPressed :
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def onBeginAnimationStep(self, deltaTime):
        ## Please feel free to add an example for a simple usage in /home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
        return 0


def createScene(rootNode):
    rootNode.findData('dt').value = '0.02'
    rootNode.findData('gravity').value = '0 0 0'
    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv
    my_RL_env = RL_env(rootNode, commandLineArguments)

    return 0
