import sys
import Sofa
import socket
import numpy as np
import geometry_util as geo

class RL_env(Sofa.PythonScriptController):
    count = 0
    HOST = ''  # Symbolic name meaning the local host
    PORT = 50007  # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    conn = 0
    data = 0
    mesh_size = 0
    visual_size = 0
    counter = 0
    state = 0
    addr = 0

    def __init__(self, node, commandLineArguments):
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : " + str(commandLineArguments))
        self.createGraph(node)


    def createGraph(self, rootNode):

        gravity = '0 0 0'
        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='CImgPlugin')
        rootNode.createObject('VisualStyle', displayFlags='showForceFields') # showCollisionModels')
        rootNode.createObject('DefaultPipeline', draw='0', depth='5', verbose='1')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('MinProximityIntersection', contactDistance='2', alarmDistance='5', name='Proximity')
        rootNode.createObject('DefaultContactManager')#, name='Response', response='FrictionContact')
        rootNode.createObject('EulerImplicitSolver', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')

        # rootNode/Cylinder
        size = 0.1
        scale = 1
        Cylinder = rootNode.createChild('Cylinder')
        self.Cylinder = Cylinder
        Cylinder.createObject('MechanicalObject', name='Cyl', template='Rigid', rx='90', dy='10', dz='25')
        Cylinder.createObject('UniformMass', totalMass='1')

        # Visual Node
        VisuNode = Cylinder.createChild('Visu_Cyl')
        VisuNode.createObject('MeshSTLLoader', name='visual_loader', filename='meshes/test_coarse.STL')
        VisuNode.createObject('OglModel', name='visual_cyl', src='@visual_loader', color='green',
                              scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
        VisuNode.createObject('RigidMapping', input='@../Cyl', output='@visual_cyl')

        # Collision Node
        CollNode = Cylinder.createChild('Coll_Cyl')
        CollNode.createObject('MeshSTLLoader', name='loader_cyl', filename='meshes/test_coarse.STL')
        CollNode.createObject('MeshTopology', src="@loader_cyl")
        CollNode.createObject('MechanicalObject', scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TPointModel')
        CollNode.createObject('RigidMapping')

        # rootNode/Cylinder
        Cylinder2 = rootNode.createChild('Cylinder2')
        self.Cylinder2 = Cylinder2
        Cylinder2.createObject('MechanicalObject', name='Cyl2', template='Rigid', rx='90', dy='10', dz='25')
        Cylinder2.createObject('UniformMass', totalMass='1')

        # Visual Node
        VisuNode2 = Cylinder2.createChild('Visu_Cyl2')
        VisuNode2.createObject('OglModel', name='visual_cyl2', fileMesh='meshes/cylinder_rot.obj', color='red',
                              scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
        VisuNode2.createObject('RigidMapping', input='@../Cyl2', output='@visual_cyl2')

        # Collision Node
        CollNode2 = Cylinder2.createChild('Coll_Cyl2')
        CollNode2.createObject('MeshObjLoader', name='loader_cyl2', filename='meshes/cylinder_rot.obj')
        CollNode2.createObject('MeshTopology', src="@loader_cyl2")
        CollNode2.createObject('MechanicalObject', scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
        CollNode2.createObject('TTriangleModel')
        CollNode2.createObject('TLineModel')
        CollNode2.createObject('TPointModel')
        CollNode2.createObject('RigidMapping')
        
        # rootNode/Cube
        dz = '20'
        Cube = rootNode.createChild('Cube')
        self.Cube = Cube
        Cube.createObject('MechanicalObject', name='Cube', template='Rigid', dz=dz)
        Cube.createObject('UniformMass', totalMass='5')

        # Visual Node
        VisuNodeCube = Cube.createChild('Visu_Cube')
        VisuNodeCube.createObject('OglModel', name='visual_cube', fileMesh='mesh/smCube27.obj', color='blue', dz=dz)
        VisuNodeCube.createObject('RigidMapping', input='@../Cube', output='@visual_cube')

        # Collision Node
        CollNodeCube = Cube.createChild('Coll_Cube')
        CollNodeCube.createObject('MeshObjLoader', name='loader_cube', filename='mesh/smCube27.obj')
        CollNodeCube.createObject('MeshTopology', src="@loader_cube")
        CollNodeCube.createObject('MechanicalObject', dz=dz)
        CollNodeCube.createObject('TTriangleModel')
#        CollNodeCube.createObject('TLineModel')
#        CollNodeCube.createObject('TPointModel')
        CollNodeCube.createObject('RigidMapping')

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
