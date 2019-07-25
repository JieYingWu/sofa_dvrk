"""
TutorialForceFieldLiverFEMPython
is based on the scene
/home/trs/sofa/src/sofa/examples/Tutorials/ForceFields/TutorialForceFieldLiverFEM.scn
but it uses the SofaPython plugin.
Further informations on the usage of the plugin can be found in
sofa/applications/plugins/SofaPython/doc/SofaPython.pdf
To launch the scene, type
runSofa /home/trs/sofa/src/sofa/examples/Tutorials/ForceFields/TutorialForceFieldLiverFEMPython.py --argv 123
The sofa python plugin might have to be added in the sofa plugin manager,
i.e. add the sofa python plugin in runSofa->Edit->PluginManager.
The arguments given after --argv can be used by accessing self.commandLineArguments, e.g. combined with ast.literal_eval to convert a string to a number.

The current file has been written by the python script
/home/trs/sofa/build/unstable//home/trs/sofa/src/sofa/applications/plugins/SofaPython/scn2python.py
Author of scn2python.py: Christoph PAULUS, christoph.paulus@inria.fr
"""

import sys
import Sofa
import rospy
import socket
import numpy as np
import geometry_util as geo
from std_msgs.msg import String

class SutureEnv(Sofa.PythonScriptController):
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
    scale = 0.1

    def __init__(self, node, commandLineArguments):
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : " + str(commandLineArguments))
        self.createGraph(node)


    def createGraph(self, rootNode):

        gravity = '0 0 -9.81'
        # rootNode
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython')
#        rootNode.createObject('VisualStyle', displayFlags='showCollisionModels')#showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst='0')
        rootNode.createObject('LCPConstraintSolver', maxIt='1000', tolerance='1e-6', mu='0.9')
        rootNode.createObject('DefaultPipeline', depth='5', verbose='0', draw='0')
        rootNode.createObject('BruteForceDetection', name='N2')

        # Who knows what's the diff betweent these
        rootNode.createObject('LocalMinDistance', alarmDistance='3', contactDistance='0.5', useLMDFilters='0')
#        rootNode.createObject('MinProximityIntersection', contactDistance='2', alarmDistance='5', name='Proximity')
        
        rootNode.createObject('DefaultContactManager')
        rootNode.createObject('DiscreteIntersection')
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')
        rootNode.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')

        # rootNode/LiverFEM
        LiverFEM = rootNode.createChild('LiverFEM')
        self.LiverFEM = LiverFEM
        LiverFEM.gravity = gravity
        LiverFEM.createObject('EulerImplicitSolver', printLog='0', rayleighStiffness='0.1', name='cg_odesolver', rayleighMass='0.1')
        LiverFEM.createObject('CGLinearSolver', threshold='1e-09', tolerance='1e-09', name='linear solver', iterations='25')
        LiverFEM.createObject('MeshTopology', name='mesh', fileTopology='mesh/liver.msh')
        LiverFEM.createObject('MechanicalObject', name='dofs', template='Vec3d', scale3d='0.005 0.005 0.005', translation='0 0 0.05')
        LiverFEM.createObject('TetrahedronFEMForceField', youngModulus='50', name='FEM', poissonRatio='0.45', template='Vec3d')
        LiverFEM.createObject('UniformMass', name='mass', totalmass='1')
        LiverFEM.createObject('FixedConstraint', indices='3 39 64', name='FixedConstraint')

        # rootNode/LiverFEM/Visu
        Visu = LiverFEM.createChild('Visu')
        self.Visu = Visu
        Visu.gravity = gravity
        Visu.createObject('OglModel', name='VisualModel', fileMesh='mesh/liver-smooth.obj', scale3d='0.005 0.005 0.005', translation='0 0 0.05')
        Visu.createObject('BarycentricMapping', input='@../dofs', name='visual mapping', output='@VisualModel')

        # rootNode/LiverFEM/Surf
        Surf = LiverFEM.createChild('Surf')
        self.Surf = Surf
        Surf.gravity = gravity
        #Surf.createObject('SphereLoader', name='SphereLoader', filename='mesh/liver.sph')
        Surf.createObject('MeshTopology', name='mesh', fileTopology='mesh/liver.msh')
        Surf.createObject('MechanicalObject', name='dofs', template='Vec3d', scale3d='0.005 0.005 0.005', translation='0 0 0.05')
        Surf.createObject('MechanicalObject', position='@[-1].position', name='mappedMS')
        Surf.createObject('TTriangleModel')#, listRadius='@[-2].listRadius', name='CollisionModel')
        Surf.createObject('BarycentricMapping', input='@../dofs', name='sphere mapping', output='@mappedMS')

        
        # # rootNode/Shaft
        # Shaft = rootNode.createChild('Shaft')
        # self.Shaft = Shaft
        # Shaft.createObject('MeshSTLLoader', name='loader', filename='meshes/link_1_shaft.stl')
        # Shaft.createObject('MeshTopology', src="@loader")
        # Shaft.createObject('MechanicalObject', name='Shaft', template='Rigid', translation='0.0002 0.0067 -0.0035', ry='90')
        # Shaft.createObject('UniformMass', totalMass='10')
        # Shaft.createObject('UncoupledConstraintCorrection', compliance='0.1')

        # # Visual Node
        # VisuShaft = Shaft.createChild('Visu_Shaft')
        # VisuShaft.createObject('OglModel', name='visual_shaft', src='@../loader')
        # VisuShaft.createObject('RigidMapping', input='@../Shaft', output='@visual_shaft')

        # # Collision Node
        # CollShaft = Shaft.createChild('Coll_Shaft')
        # CollShaft.createObject('MechanicalObject', name='mecha_shaft', src='@../loader')
        # CollShaft.createObject('TTriangleModel')
        # CollShaft.createObject('TLineModel')
        # CollShaft.createObject('TPointModel')
        # CollShaft.createObject('RigidMapping', input='@../Shaft', output='@mecha_shaft')

        # # rootNode/Link1
        # Link1 = rootNode.createChild('Link1')
        # self.Link1 = Link1
        # Link1.createObject('MeshSTLLoader', name='loader', filename='meshes/link_1.stl')
        # Link1.createObject('MeshTopology', src="@loader")
        # Link1.createObject('MechanicalObject', name='Link1', template='Rigid', translation='0 -0.016 0.0009', rx='90')
        # Link1.createObject('UniformMass', totalMass='10')
        # Link1.createObject('UncoupledConstraintCorrection', compliance='0.1')

        # # Visual Node
        # VisuLink1 = Link1.createChild('Visu_Link1')
        # VisuLink1.createObject('OglModel', name='visual_link1', src='@../loader')
        # VisuLink1.createObject('RigidMapping', input='@../Link1', output='@visual_link1')

        # # Collision Node
        # CollLink1 = Link1.createChild('Coll_Link1')
        # CollLink1.createObject('MechanicalObject', name='mecha_link1', src='@../loader')
        # CollLink1.createObject('TTriangleModel')
        # CollLink1.createObject('TLineModel')
        # CollLink1.createObject('TPointModel')
        # CollLink1.createObject('RigidMapping', input='@../Link1', output='@mecha_link1')

        # rootNode/Link2
        Link2L = rootNode.createChild('Link2L')
        self.Link2L = Link2L
        Link2L.createObject('MeshSTLLoader', name='loader', filename='meshes/link_2.stl')
        Link2L.createObject('MeshTopology', src="@loader")
        Link2L.createObject('MechanicalObject', name='Link2L', template='Rigid')
        Link2L.createObject('UniformMass', totalMass='10')
        Link2L.createObject('UncoupledConstraintCorrection', compliance='0.1')
#        Link2L.createObject('FixedConstraint', indices='0')
#        Link2L.createObject('FixedRotationConstraint', template='Rigid', FixedXRotation='1', FixedYRotation='1', FixedZRotation='0')

        # Visual Node
        VisuLink2L = Link2L.createChild('Visu_Link2L')
        VisuLink2L.createObject('OglModel', name='visual_link2L', src='@../loader')
        VisuLink2L.createObject('RigidMapping', input='@../Link2L', output='@visual_link2L')

        # Collision Node
        CollLink2L = Link2L.createChild('Coll_Link2L')
        CollLink2L.createObject('MechanicalObject', name='mecha_link2L', src='@../loader')
        CollLink2L.createObject('TTriangleModel')
        CollLink2L.createObject('TLineModel')
        CollLink2L.createObject('TPointModel')
        CollLink2L.createObject('RigidMapping', input='@../Link2L', output='@mecha_link2L')

        # # rootNode/Link2R
        # Link2R = rootNode.createChild('Link2R')
        # self.Link2R = Link2R
        # Link2R.createObject('MeshSTLLoader', name='loader', filename='meshes/link_2.stl')
        # Link2R.createObject('MeshTopology', src="@loader")
        # Link2R.createObject('MechanicalObject', name='Link2R', template='Rigid', translation='0 0 -0.0025', ry='180')
        # Link2R.createObject('UniformMass', totalMass='10')
        # Link2R.createObject('UncoupledConstraintCorrection', compliance='0.1')
        # Link2R.createObject('FixedConstraint', indices='0')
        # Link2R.createObject('FixedRotationConstraint', template='Rigid', FixedXRotation='1', FixedYRotation='1')

        # # Visual Node
        # VisuLink2R = Link2R.createChild('Visu_Link2R')
        # VisuLink2R.createObject('OglModel', name='visual_link2R', src='@../loader')
        # VisuLink2R.createObject('RigidMapping', input='@../Link2R', output='@visual_link2R')

        # # Collision Node
        # CollLink2R = Link2R.createChild('Coll_Link2R')
        # CollLink2R.createObject('MechanicalObject', name='mecha_link2R', src='@../loader')
        # CollLink2R.createObject('TTriangleModel')
        # CollLink2R.createObject('TLineModel')
        # CollLink2R.createObject('TPointModel')
        # CollLink2R.createObject('RigidMapping', input='@../Link2R', output='@mecha_link2R')
        
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
        vel = np.array(self.Link2L.getObject('Link2L').velocity)
        print(vel.shape)
        if c == "Q":
            newVel = vel + np.array([self.scale, 0, 0, 0, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = newVel
        if c == "W":
            newVel = vel + np.array([-self.scale, 0, 0, 0, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = newVel
        if c == "A":
            newVel = vel + np.array([0, self.scale, 0, 0, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = newVel
        if c == "D":
            newVel = vel + np.array([0, -self.scale, 0, 0, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = newVel
        if c == "Z":
            newVel = vel + np.array([0, 0, self.scale, 0, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = newVel
        if c == "X":
            newVel = vel + np.array([0, 0, -self.scale, 0, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = newVel
        if c == "T":
            newVel = vel + np.array([0, 0, 0, self.scale, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = geo.arrToStr(newVel)
        if c == "Y":
            newVel = vel + np.array([0, 0, 0, -self.scale, 0, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = geo.arrToStr(newVel)
        if c == "G":
            newVel = vel + np.array([0, 0, 0, 0, self.scale, 0])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = geo.arrToStr(newVel)
        if c == "H":
            newVel = vel + np.array([0, 0, 0, 0, -self.scale, 0])
            print(newVel.shape)
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = geo.arrToStr(newVel)
        if c == "V":
            newVel = vel + np.array([0, 0, 0, 0, 0, self.scale])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = geo.arrToStr(newVel)
        if c == "B":
            newVel = vel + np.array([0, 0, 0, 0, 0, -self.scale])
            newVel = geo.arrToStr(newVel)
            self.Link2L.getObject('Link2L').velocity = geo.arrToStr(newVel)
        if c == "E":
            self.Link2L.getObject('Link2L').velocity = '0 0 0 0 0 0'

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


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()
    
    
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
