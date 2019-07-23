import sys
import Sofa
import numpy as np
import cisstRobotPython
import geometry_util as geo
#from std_msgs.msg import String

class SutureEnv(Sofa.PythonScriptController):

    robot = cisstRobotPython.robManipulator()
    joints = np.zeros(6)
    size = 3
    scale = 1
    step = 0.005
    axis_scale=100
    
    def __init__(self, node, commandLineArguments):
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : " + str(commandLineArguments))
#        self.joints = np.array([0.707, 0.707, 0.707, 0.707, 0.707, 0.707])
        self.joints = np.array([3.1416, 0.0, 0.5, 0.0, 0.0, 0.0])

        self.robot.LoadRobot('/home/jieying/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob')

        self.createGraph(node)


    def createGraph(self, rootNode):
        
        gravity = '0 0 0'
        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels')# showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst='0')
        rootNode.createObject('LCPConstraintSolver', maxIt='1000', tolerance='1e-6', mu='0.9')
        rootNode.createObject('DefaultPipeline', depth='5', verbose='0', draw='0')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('MinProximityIntersection', contactDistance='0.5', alarmDistance='3')
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
        rootNode.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')
        
        # rootNode/Needle
        pose = self.robot.ForwardKinematics(self.joints)
        needle_pos = geo.matToPos(pose)

        Needle = rootNode.createChild('Needle')
        self.Needle = Needle
        Needle.createObject('MeshSTLLoader', name='loader', filename='meshes/test_coarse.STL', triangulate='true', scale=self.size)
        Needle.createObject('MechanicalObject', name='Nee', template='Rigid', position=needle_pos)
        Needle.createObject('UniformMass', totalMass='1000', showAxisSizeFactor=str(self.axis_scale))
        Needle.createObject('UncoupledConstraintCorrection', compliance='1')

        # Visual Node
        VisuNeedle = Needle.createChild('Visu_Nee')
        VisuNeedle.createObject('OglModel', name='visual_nee', src='@../loader', color='blue')
        VisuNeedle.createObject('RigidMapping', input='@../Nee', output='@visual_nee')

        # Collision Node
        CollNeedle = Needle.createChild('Coll_Needle')
        CollNeedle.createObject('MeshTopology', src="@../loader")
        CollNeedle.createObject('MechanicalObject', name='coll', src="@../loader")
        CollNeedle.createObject('TTriangleModel')
        CollNeedle.createObject('TLineModel')
        CollNeedle.createObject('TPointModel')
        CollNeedle.createObject('RigidMapping', src='@../Nee', output='@coll')

        # rootNode/Phantom
        Phantom = rootNode.createChild('Phantom')
        self.Phantom = Phantom
        Phantom.createObject('MeshSTLLoader', name='loader', filename='meshes/Suture_hex.STL')
        Phantom.createObject('TriangleSetTopologyContainer', name='topo', src='@loader')
        Phantom.createObject('TriangleSetGeometryAlgorithms', template='Vec3d', recomputeTrianglesOrientation='1')        
        Phantom.createObject('MechanicalObject', name='Pha', src='@loader', template='Vec3d',
                             scale3d=str(self.size) + ' ' + str(self.scale * self.size) + ' '
                             + str(self.size), rx='90', translation='500 10 20')
#        Phantom.createObject('DiagonalMass', massDensity='0.01')
        Phantom.createObject('MeshMatrixMass', totalMass='10')
        Phantom.createObject('FixedConstraint', indices='8 40 252 407')
        Phantom.createObject('TriangularFEMForceField', youngModulus='10000', template='Vec3d', poissonRatio='0.1', method='large')
        Phantom.createObject('TriangularBendingSprings', stiffness='1', template='Vec3d', damping='10.0')
        Phantom.createObject('UncoupledConstraintCorrection', compliance='1')

        # Visual Node
        VisuPhantom = Phantom.createChild('Visu_Pha')
        VisuPhantom.createObject('OglModel', name='visual_pha',src='@../loader', color='red',
                              scale3d=str(self.size) + ' ' + str(self.scale * self.size) + ' ' + str(self.size))
        VisuPhantom.createObject('IdentityMapping', input='@../Pha', output='@visual_pha')

        # Collision Node
        CollPhantom = Phantom.createChild('Triangular_Surf')
        CollPhantom.createObject('MechanicalObject', name='coll_pha', src='@../loader', template='Vec3d')
#                            scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
#        CollPhantom.createObject('TriangleSetTopologyContainer', name='Container')
#        CollPhantom.createObject('TriangleSetTopologyModifier', name='Modifier')
#        CollPhantom.createObject('Tetra2TriangleTopologicalMapping', input='@../topo', output='@Container')
        CollPhantom.createObject('TPointModel')
        CollPhantom.createObject('TLineModel')
        CollPhantom.createObject('TTriangleModel')
        CollPhantom.createObject('IdentityMapping', input='@../Pha', output='@coll_pha')        
        # rootNode/Link0
        pose = self.robot.ForwardKinematics(self.joints, N=0)
        link0_pos = geo.matToPos(pose)
        
        Link0 = rootNode.createChild('Link0')
        self.Link0 = Link0
        Link0.createObject('MechanicalObject', name='mecha', template='Rigid', position=link0_pos)
        Link0.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))
        # rootNode/Link1
        pose = self.robot.ForwardKinematics(self.joints, N=1)
        link1_pos = geo.matToPos(pose)
        
        Link1 = rootNode.createChild('Link1')
        self.Link1 = Link1
        Link1.createObject('MechanicalObject', name='mecha', template='Rigid', position=link1_pos)
        Link1.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))
        # rootNode/Link2
        pose = self.robot.ForwardKinematics(self.joints, N=2)
        link2_pos = geo.matToPos(pose)
        
        Link2 = rootNode.createChild('Link2')
        self.Link2 = Link2
        Link2.createObject('MechanicalObject', name='mecha', template='Rigid', position=link2_pos)
        Link2.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))
        # rootNode/Link3
        pose = self.robot.ForwardKinematics(self.joints, N=3)
        link3_pos = geo.matToPos(pose)
        
        Link3 = rootNode.createChild('Link3')
        self.Link3 = Link3
        Link3.createObject('MechanicalObject', name='mecha', template='Rigid', position=link3_pos)
        Link3.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))
        # rootNode/Link4
        pose = self.robot.ForwardKinematics(self.joints, N=4)
        link4_pos = geo.matToPos(pose)
        
        Link4 = rootNode.createChild('Link4')
        self.Link4 = Link4
        Link4.createObject('MechanicalObject', name='mecha', template='Rigid', position=link4_pos)
        Link4.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))
        # rootNode/Link5
        pose = self.robot.ForwardKinematics(self.joints, N=5)
        link5_pos = geo.matToPos(pose)
        
        Link5 = rootNode.createChild('Link5')
        self.Link5 = Link5
        Link5.createObject('MechanicalObject', name='mecha', template='Rigid', position=link5_pos)
        Link5.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))
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

    def updateRobot(self):
        needlePos = self.robot.ForwardKinematics(self.joints)
        needlePos = geo.matToPos(needlePos)
        l0Pos = self.robot.ForwardKinematics(self.joints, N=0)
        l0Pos = geo.matToPos(l0Pos)
        l1Pos = self.robot.ForwardKinematics(self.joints, N=1)
        l1Pos = geo.matToPos(l1Pos)
        l2Pos = self.robot.ForwardKinematics(self.joints, N=2)
        l2Pos = geo.matToPos(l2Pos)
        l3Pos = self.robot.ForwardKinematics(self.joints, N=3)
        l3Pos = geo.matToPos(l3Pos)
        l4Pos = self.robot.ForwardKinematics(self.joints, N=4)
        l4Pos = geo.matToPos(l4Pos)
        l5Pos = self.robot.ForwardKinematics(self.joints, N=5)
        l5Pos = geo.matToPos(l5Pos)
        
        self.Needle.getObject('Nee').position = needlePos
        self.Link0.getObject('mecha').position = l0Pos
        self.Link1.getObject('mecha').position = l1Pos
        self.Link2.getObject('mecha').position = l2Pos
        self.Link3.getObject('mecha').position = l3Pos
        self.Link4.getObject('mecha').position = l4Pos
        self.Link5.getObject('mecha').position = l5Pos
        

    # Note: Hold control when key is pressed
    def onKeyPressed(self, c):
        ## usage e.g.
        if c == "Q":
            self.joints[0] += self.step
        if c == "W":
            self.joints[0] -= self.step
        if c == "A":
            self.joints[1] += self.step
        if c == "D":
            self.joints[1] -= self.step
        if c == "Z":
            self.joints[2] += self.step
        if c == "X":
            self.joints[2] -= self.step
        if c == "T":
            self.joints[3] += self.step
        if c == "Y":
            self.joints[3] -= self.step
        if c == "G":
            self.joints[4] += self.step
        if c == "H":
            self.joints[4] -= self.step
        if c == "V":
            self.joints[5] += self.step
        if c == "B":
            self.joints[5] -= self.step
        if c == "E":
            self.Needle.getObject('Nee').velocity = '0 0 0 0 0 0'

        self.updateRobot()
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


# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#
#
# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("chatter", String, callback)
#     rospy.spin()
    
    
def createScene(rootNode):
    rootNode.findData('dt').value = '0.02'
    rootNode.findData('gravity').value = '0 0 0'
#    listener()
    
    try:
        sys.argv[0]
    except:
        commandLineArguments = []
    else:
        commandLineArguments = sys.argv
    my_suture_env = SutureEnv(rootNode, commandLineArguments)

    return 0
