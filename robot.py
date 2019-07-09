import sys
import Sofa
import numpy as np
import cisstRobotPython
import geometry_util as geo
from urdf_parser_py import urdf  # noqa
import urdf_parser_py.xml_reflection as xmlr
from pykdl_utils.kdl_kinematics import KDLKinematics

class RobotEnv(Sofa.PythonScriptController):
    axis_scale = 0.2
    
    def __init__(self, node, commandLineArguments):
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : " + str(commandLineArguments))
        self.createGraph(node)


    def createGraph(self, rootNode):

        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython')
        rootNode.createObject('VisualStyle', displayFlags='showVisual showBehaviorModels')#  showForceFields showCollision showMapping')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst='0')
        rootNode.createObject('LCPConstraintSolver', maxIt='1000', tolerance='1e-6', mu='0.9')
        rootNode.createObject('DefaultPipeline', depth='5', verbose='0', draw='0')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('MinProximityIntersection', contactDistance='0.5', alarmDistance='3')
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
        rootNode.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1e-8', tolerance='1e-5', name='linearSolver', iterations='25')

        
#        f = open('meshes/psm1.urdf')
#        robot = urdf.URDF.from_xml_string(f.read())
#        f.close()
#        print(robot.link_map.keys())
#        print(robot.get_chain('world', 'PSM1_tool_tip_link', links=False))
#        kin = KDLKinematics(robot, 'world', 'PSM1_tool_tip_link')
        #q = kin.random_joint_angles()

        r = cisstRobotPython.robManipulator()
        r.LoadRobot('/home/jieying/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/deprecated/dvpsm.rob')
#        q = np.array([0.707, 0.707, 0.707, 0.707, 0.707, 0.707])
        q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # rootNode/BaseLink
        pose = r.ForwardKinematics(q, N=0)
        base_pos = geo.matToPos(pose)

        BaseLink = rootNode.createChild('BaseLink')
        self.BaseLink = BaseLink
        BaseLink.createObject('MeshSTLLoader', name='loader', filename='meshes/psm/psm_base.stl')
        BaseLink.createObject('MechanicalObject', name='mecha', template='Rigid', translation='0 0 0', rotation='0 0 0 0', position='0 0 0 0 0 0 1')#geo.posToStr('0.039 -0.40788 -0.07879 -90.0 0.0 0.0'))
        BaseLink.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))

        # Visual Node
        VisuBase = BaseLink.createChild('VisuBase')
        VisuBase.createObject('OglModel', name='visual', src='@../loader')#,
                              #scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
        VisuBase.createObject('RigidMapping', input='@../mecha', output='@visual')
        
        # # rootNode/Link1
        pose = r.ForwardKinematics(q, N=1)
        link1_pos = geo.matToTrans(pose)
        link1_rot = geo.matToRot(pose)
        
        Link1 = rootNode.createChild('Link1')
        self.Link1 = Link1
        Link1.createObject('MeshSTLLoader', name='loader', filename='meshes/psm/outer_yaw.stl')
        Link1.createObject('MechanicalObject', name='mecha', template='Rigid', translation=link1_pos, rotation=link1_rot, position=geo.posToStr('0.0125 -0.08 0.2965 0.0 -90.0 -90.0'))
        Link1.createObject('UniformMass', totalMass='1', template='Rigid', showAxisSizeFactor=str(self.axis_scale))

        # Visual Node
        VisuLink1 = Link1.createChild('VisuLink1')
        VisuLink1.createObject('OglModel', name='visual', src='@../loader')#,
                              #scale3d=str(size) + ' ' + str(scale * size) + ' ' + str(size))
        VisuLink1.createObject('RigidMapping', input='@../mecha', output='@visual')

        
        
        # print(link1_pos)
        # print(link2_pos)
        # print(tool_pos)
        # print(tip_pos)
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
        return 0

    # Note: Hold control when key is pressed
    def onKeyPressed(self, c):
        return 0

    def onMouseWheel(self, mouseX, mouseY, wheelDelta):
        ## usage e.g.
        # if isPressed :
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0

    def storeResetState(self):
        return 0

    def cleanup(self):
        # self.conn.close()
        return 0

    def onGUIEvent(self, strControlID, valueName, strValue):
        print
        'onGUIEvent'
        return 0

    def onEndAnimationStep(self, deltaTime):
        return 0

    def onLoaded(self, node):
        return 0

    def reset(self):
        return 0

    def onMouseButtonMiddle(self, mouseX, mouseY, isPressed):
        ## usage e.g.
        # if isPressed :
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def bwdInitGraph(self, node):
        return 0

    def onScriptEvent(self, senderNode, eventName, data):
        print
        'onScriptEvent'
        return 0

    def onMouseButtonRight(self, mouseX, mouseY, isPressed):
        ## usage e.g.
        # if isPressed :
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0

    def onBeginAnimationStep(self, deltaTime):
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
    my_robot_env = RobotEnv(rootNode, commandLineArguments)

    return 0
