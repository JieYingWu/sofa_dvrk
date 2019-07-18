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


    def populateNonMoving(self, node, filename, translation=[0, 0, 0], rotation=[0, 0, 0], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('MeshSTLLoader', name='loader', filename=filename)
        node.createObject('MeshTopology', src='@loader')
        node.createObject('MechanicalObject', name='mecha', src='@loader', scale3d=scale, translation=translation, rotation=rotation)
        node.createObject('TTriangleModel', simulated=0, moving=0)
        node.createObject('TLineModel', simulated=0, moving=0)
        node.createObject('TPointModel', simulated=0, moving=0)
        node.createObject('OglModel', name='visu', src='@loader', scale3d=scale, translation=translation, rotation=rotation, color=color)
        

    def populateRigid(self, node, filename, translation=[0, 0, 0], rotation=[0, 0, 0], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        node.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='15')
        if (filename[-4:] == '.obj'):
            node.createObject('MeshObjLoader', name='loader', filename=filename)
        elif (filename[-4:] == '.STL' or filename[-4:] == '.stl'):
            node.createObject('MeshSTLLoader', name='loader', filename=filename)
        node.createObject('MechanicalObject', name='mecha', template='Rigid3d', scale3d=scale, translation=translation, rotation=rotation)
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

    def populateVec(self, node, filename, translation=[0, 0, 0], rotation=[0, 0, 0], scale=[1, 1, 1], mass=1.0, color='red'):
        node.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        node.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='15')
        if (filename[-4:] == '.obj'):
            node.createObject('MeshObjLoader', name='loader', filename=filename)
        elif (filename[-4:] == '.STL' or filename[-4:] == '.stl'):
            node.createObject('MeshSTLLoader', name='loader', filename=filename)
        node.createObject('MeshTopology', src='@loader')
        node.createObject('MechanicalObject', name='mecha', template='Vec3d', scale3d=scale, translation=translation, rotation=rotation)
        node.createObject('TriangularFEMForceField', youngModulus=1e12, poissonRatio=0)
        node.createObject('UniformMass', totalMass=mass)
        node.createObject('UncoupledConstraintCorrection')

        # Visual Node
        VisuNode = node.createChild('Visu_Cyl')
        VisuNode.createObject('OglModel', name='visual', src='@../loader', color=color, scale3d=scale, translation=translation, rotation=rotation)
        VisuNode.createObject('IdentityMapping', input='@../mecha', output='@visual')

        # Collision Node
        CollNode = node.createChild('Coll_Cyl')
        CollNode.createObject('MeshTopology', src="@../loader")
        CollNode.createObject('MechanicalObject', src='@../loader', scale3d=scale, translation=translation, rotation=rotation)
        CollNode.createObject('TPointModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('IdentityMapping', input='@../mecha', output='@.')

        return 0
    
    def populateSpring(self, spring, translation, rotation, color='gray'):
        index = [29575, 29040, 29597]
        spring.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.03', name='odesolver', rayleighMass='1')
        spring.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='20')
        spring.createObject('MeshObjLoader', name='loader', filename='meshes/steel_extension_spring.obj')
        spring.createObject('MeshTopology', name='topo', src='@loader')
#        spring.createObject('EdgeSetGeometryAlgorithms')
#        spring.createObject('TetrahedronSetTopologyContainer', src='@loader', fileTopology='meshes/steel_extension_spring.msh')
#        spring.createObject('TetrahedronSetGeometryAlgorithms', template='CudaVec3f')
        
        spring.createObject('SparseGridTopology', n='10 10 10', src='@topo')
        spring.createObject('MechanicalObject', template='Vec3d', name='spring', rotation=rotation, translation=translation)
        spring.createObject('TetrahedronFEMForceField', youngModulus='1e3', poissonRatio='0.26')
#        spring.createObject('MeshSpringForceField', stiffness='10000', damping='1')
        spring.createObject('UniformMass', totalMass=1.0, showAxisSizeFactor=100)
#        spring.createObject('FixedConstraint', indices=index, name='FixedConstraint')
#        spring.createObject('UncoupledConstraintCorrection')

        # rootNode/Spring/VisuSpring
        VisuSpring = spring.createChild('VisuSpring')
        VisuSpring.createObject('OglModel', name='visu', src='@../loader', color=color)
        VisuSpring.createObject('BarycentricMapping', input='@..', output='@visu')

        # rootNode/Spring/CollSpring
        CollSpring = spring.createChild('CollSpring')
        CollSpring.createObject('MeshTopology', name='topo', src='@../loader')
        CollSpring.createObject('MechanicalObject', name='coll', src='@../loader')
        CollSpring.createObject('TTriangleModel')
        CollSpring.createObject('TLineModel')
        CollSpring.createObject('TPointModel')
        CollSpring.createObject('BarycentricMapping', input='@..', output='@.')
        
        return 0
    
    def createGraph(self,rootNode):
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython SofaOpenglVisual SofaCUDA')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels')# showCollisionModels showInteractionForceFields showForceFields')
#        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
#        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-6, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)
        rootNode.createObject('BruteForceDetection')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=0.8)
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager')
        # rootNode.createObject('AnimationLoopParallelScheduler', threadNumber=2)


        # rootNode/Floor
#        scale=[10, 1, 10]
#        translation = [100, 0, 100]
        # Floor = rootNode.createChild('Floor')
        # self.Floor = Floor
        # Floor.createObject('MeshObjLoader', name='loader', filename='mesh/floorFlat.obj')
        # Floor.createObject('MeshTopology', src='@loader')
        # Floor.createObject('MechanicalObject', name='mecha', src='@loader', scale3d=scale)
        # Floor.createObject('TTriangleModel', simulated=0, moving=0)
        # Floor.createObject('TLineModel', simulated=0, moving=0)
        # Floor.createObject('TPointModel', simulated=0, moving=0)
        # Floor.createObject('OglModel', name='visu', src='@loader', scale3d=scale)

        tableWidth = 95.7
        tableHeight = 72
        
        # rootNode/Support0
        supportHeight = 95.7
        offset = tableWidth/2+16
        height = supportHeight/2
        translation = [-offset, height, 0]
        Support0 = rootNode.createChild('Support0')
        self.populateNonMoving(Support0, 'meshes/lego_support.STL', translation=translation, color='blue')
        self.Support0 = Support0

        # rootNode/Support1
        translation = [0, height, -offset]
        rotation = [0, 90, 0]
        Support1 = rootNode.createChild('Support1')
        self.populateNonMoving(Support1, 'meshes/lego_support.STL', translation=translation, rotation=rotation, color='blue')
        self.Support1 = Support1

        # rootNode/Support2
        translation = [offset, height, 0]
        rotation = [0, 180, 0]
        Support2 = rootNode.createChild('Support2')
        self.populateNonMoving(Support2, 'meshes/lego_support.STL', translation=translation, rotation=rotation, color='blue')
        self.Support2 = Support2

        # rootNode/Support3
        translation = [0, height, offset]
        rotation = [0, 270, 0]
        Support3 = rootNode.createChild('Support3')
        self.populateNonMoving(Support3, 'meshes/lego_support.STL', translation=translation, rotation=rotation, color='blue')
        self.Support3 = Support3

        # rootNode/Tabletop
        translation = [0, tableHeight, 0]
        Tabletop = rootNode.createChild('Tabletop')
        self.populateVec(Tabletop, 'meshes/lego_platform.STL', translation=translation, mass=100.0, color='green')
        self.Tabletop = Tabletop

        # rootNode/Spring0
        offset = tableWidth/2+3.5
        springAngle = 32
        springLength = 38.1
        springHeight = tableHeight - 0.9  # + springLength * math.sin(springAngle*math.pi/180) / 2
        translation = [-offset, springHeight, 0]
        rotation = [0, 0, -springAngle]
        Spring0 = rootNode.createChild('Spring0')
        self.populateSpring(Spring0, translation, rotation)
        self.Spring0 = Spring0

        # rootNode/Spring1
        translation=[0, springHeight, -offset]
        # Euler angle in XYZ form
        rotation = [-90, 270-springAngle, 90]
        Spring1 = rootNode.createChild('Spring1')
        self.populateSpring(Spring1, translation, rotation, color='red')
        self.Spring1 = Spring1

        # rootNode/Spring2
        translation=[offset, springHeight, 0]
        rotation = [0, 0, 180+springAngle]
        Spring2 = rootNode.createChild('Spring2')
        self.populateSpring(Spring2, translation, rotation, color='yellow')
        self.Spring2 = Spring2

        # rootNode/Spring3
        translation=[0, springHeight, offset]
        rotation = [90, 90+springAngle, 90]
        Spring3 = rootNode.createChild('Spring3')
        self.populateSpring(Spring3, translation, rotation, color='blue')
        self.Spring3 = Spring3

#        rootNode.createObject('AttachConstraint', object1='@Tabletop', object2='@Spring0', twoWay='true', indices1='159', indices2='292', constraintFactor='1')
#        rootNode.createObject('AttachConstraint', object1='@Tabletop', object2='@Spring1', twoWay='true', indices1='212', indices2='292', constraintFactor='1')
#        rootNode.createObject('AttachConstraint', object1='@Tabletop', object2='@Spring2', twoWay='true', indices1='246', indices2='292', constraintFactor='1')
#        rootNode.createObject('AttachConstraint', object1='@Tabletop', object2='@Spring3', twoWay='true', indices1='90', indices2='292', constraintFactor='1')
        
        # rootNode/Cylinder
        # scale = [0.8, 0.8, 0.8]
        # translation = [22, 70, -48]
        # Cylinder = rootNode.createChild('Cylinder')
        # self.populateRigid(Cylinder, 'meshes/cylinder_rot.obj', translation=translation, scale=scale, mass=1e5, color='blue')
        # self.Cylinder = Cylinder
        
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
    rootNode.findData('dt').value = '0.001'
    rootNode.findData('gravity').value = '0 -1000 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = SpringEnv(rootNode,commandLineArguments)
    
    return 0
