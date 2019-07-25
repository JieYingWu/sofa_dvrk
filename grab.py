import sys
import Sofa

class baseModel (Sofa.PythonScriptController):
    counter = 0;
    state = 0;

    def __init__(self, node, commandLineArguments) : 
        self.commandLineArguments = commandLineArguments
        print "Command line arguments for python : "+str(commandLineArguments)
        self.createGraph(node)
        return None;

    def createGraph(self,rootNode):

        # rootNode
        self.rootNode = rootNode
        rootNode.createObject('RequiredPlugin', pluginName='CImgPlugin SofaMiscCollision SofaPython')
        rootNode.createObject('VisualStyle', displayFlags='showForceFields showCollisionModels')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst='0')
        rootNode.createObject('LCPConstraintSolver', mu='1e-9', maxIt='1000', printLog='0', initial_guess='false', build_lcp='false', tolerance='1e-3')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('DefaultPipeline', draw='1', depth='5', verbose='0')
        rootNode.createObject('DefaultContactManager', response='FrictionlessContact', name='collision response')
        rootNode.createObject('MinProximityIntersection', contactDistance='0.1', alarmDistance='0.2', name='Proximity')
        rootNode.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='cg_odesolver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1.0e-9', tolerance='1.0e-9', name='linear solver', iterations='25')
        
        # rootNode/arm_1
        arm_1 = rootNode.createChild('arm_1')
        self.arm_1 = arm_1
        arm_1.createObject('MeshSTLLoader', name='loader_stl', filename='meshes/arm_1.STL')
        arm_1.createObject('MeshTopology', src='@loader_stl', name='loader')
        arm_1.createObject('MechanicalObject', name='Arm_1', template='Rigid', dx='-100', dy='-20', dz='-50', velocity='0 -10 0 0 0 0')
        # arm_1.createObject('MechanicalObject', name='Arm_1', template='Rigid', velocity = '0 -0.5 0 0 0 0')
        arm_1.createObject('UniformMass', name='Mass', totalMass='1000000')
        arm_1.createObject('UncoupledConstraintCorrection', compliance='0.1')

        # Collision Node
        CollNode1 = arm_1.createChild('coll_arm_1')
        CollNode1.createObject('MechanicalObject',name='arm_coll', src='@../loader_stl')
        # CollNode1.createObject('MeshSpringForceField', stiffness='100000')
        CollNode1.createObject('TTriangleModel')
        CollNode1.createObject('TLineModel')
        CollNode1.createObject('TPointModel')
#        CollNode1.createObject('TSphereModel', radius='2')
        CollNode1.createObject('RigidMapping', input='@../Arm_1', output='@arm_coll')
        # CollNode1.createObject('IdentityMapping')
        # CollNode1.createObject('BarycentricMapping')

        # Arm_1 in one model for collision testing
        # arm_1 = rootNode.createChild('arm_1')
        # self.arm_1 = arm_1
        # arm_1.createObject('MeshSTLLoader', name='loader_stl', filename='meshes/arm_1.STL')
        # # arm_1.createObject('Mesh', src='@loader_stl')
        # arm_1.createObject('MeshTopology', src='@loader_stl', name='loader')
        # arm_1.createObject('MechanicalObject', name='Arm_1', template='Rigid', velocity = '0 -1 0')
        # arm_1.createObject('UniformMass', name='Mass', totalMass='1')
        # # arm_1.createObject('MeshSpringForceField', name='Springs', stiffness='2000', damping='1')
        # # arm_1.createObject('DiagonalMass', massDensity='1')
        # arm_1.createObject('TTriangleModel')
        # arm_1.createObject('TLineModel')
        # arm_1.createObject('TPointModel')


        # rootNode/arm_2
        arm_2 = rootNode.createChild('arm_2')
        self.arm_2 = arm_2
        arm_2.createObject('MechanicalObject', name='Arm_2', template='Rigid', dy='-20', velocity = '0 1 0 0 0 0')
        arm_2.createObject('UniformMass', name='Mass', totalMass='1')

        # Collision Node
        CollNode2 = arm_2.createChild('Coll_arm_2')
        CollNode2.createObject('MeshSTLLoader', name='loader_stl', filename='meshes/arm_2.STL')
        CollNode2.createObject('MeshTopology', src='@loader_stl', name='loader')
        CollNode2.createObject('MechanicalObject', name='Arm_2')
        # arm_1.createObject('FixedRotationConstraint', FixedXRotation='0', FixedYRotation='0', FixedZRotation='1')
        CollNode2.createObject('TTriangleModel')
        CollNode2.createObject('TLineModel')
        CollNode2.createObject('TPointModel')
        CollNode2.createObject('RigidMapping')

        # rootNode/Center
        Center = rootNode.createChild('Center')
        self.Center = Center
        Center.createObject('MeshSTLLoader', name='loader_stl', filename='meshes/center.STL')
        Center.createObject('MeshTopology', src='@loader_stl')
        Center.createObject('MechanicalObject', name='Center', template='Vec3d', src='@loader_stl')
        Center.createObject('UniformMass', totalMass='1')
#        Center.createObject('FixedConstraint', indices='0 1 2 3')
        Center.createObject('TriangularFEMForceField', youngModulus='1000000', template='Vec3d', poissonRatio='0.1', method='large')
        Center.createObject('TriangularBendingSprings', stiffness='4', template='Vec3d', damping='2.0')
        Center.createObject('UncoupledConstraintCorrection', compliance='1')
#        Center.createObject('ConstantForceField', indices='0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16', forces='0 -10 0')

        Center.createObject('MeshSpringForceField', stiffness='100000000')
#        Center.createObject('StiffSpringForceField', stiffness='100')
#        Center.createObject('RestShapeSpringsForceField', name='Springs', stiffness='0.1')

        # Collision Node
        CollCenter = Center.createChild('CollCenter')
        CollCenter.createObject('MechanicalObject', name='coll_center', src='@../loader_stl', template='Vec3d')
        CollCenter.createObject('TTriangleModel')
        CollCenter.createObject('TLineModel')
        CollCenter.createObject('TPointModel')
        # Center.createObject('TSphereModel')
        CollCenter.createObject('IdentityMapping', input='@../Center', output='@coll_center')      

        return 0;

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onKeyReleased(self, c):
        ## usage e.g.
        #if c=="A" :
        #    print "You released a"
        return 0;

    def initGraph(self, node):
        return 0;

    def onKeyPressed(self, c):
        ## usage e.g.
        if c == "E":
           self.arm_1.getObject('Arm_1').velocity = '0 0 0 0 0 0'
           print str(self.arm_1.getObject('Arm_1').findData('velocity').value)
        #    print str(self.arm_1.Coll_arm_1.getObject('arm_coll').findData('velocity').value)
        return 0;

    def onMouseWheel(self, mouseX,mouseY,wheelDelta):
        ## usage e.g.
        #if isPressed : 
        #    print "Control button pressed+mouse wheel turned at position "+str(mouseX)+", "+str(mouseY)+", wheel delta"+str(wheelDelta)
        return 0;

    def storeResetState(self):
        return 0;

    def cleanup(self):
        return 0;

    def onGUIEvent(self, strControlID,valueName,strValue):
        return 0;

    def onEndAnimationStep(self, deltaTime):
        return 0;

    def onLoaded(self, node):
        return 0;

    def reset(self):
        return 0;

    def onMouseButtonMiddle(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Middle mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def bwdInitGraph(self, node):
        # f1 = open("measurements/pos_rigidModel_readout.txt","w+")
        # f2 = open("measurements/pos_collModel_readout.txt","w+")
        # position = self.arm_1.getObject('Arm_1').position
        # print str(len(position))
        # # position = self.arm_1.CollNode1.getObject('arm_coll').findData('position').value
        # position = self.arm_1.coll_arm_1.getObject('arm_coll').position
        # self.f = open("measurements/velocity_4.txt","w+")
        
        

        return 0;

    def onScriptEvent(self, senderNode, eventName,data):
        return 0;

    def onMouseButtonRight(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Right mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onBeginAnimationStep(self, deltaTime):
        # change the veloctiy of the different arms to the speed where they more inside, after the registration of the contact, stop the velocity and move the arms in different direction
        
        # if (self.counter == 50):
        #     velocity = self.arm_1.coll_arm_1.getObject('arm_coll').velocity
        #     print str(len(velocity))
        #     # change the position by a small increment
        #     # self.arm_1.getObject('Arm_1').velocity = '0 0 0 0 0 0'
        #     # self.arm_1.getObject('Arm_1').velocity
        #     # self.arm_1.Coll_arm_1.getObject('arm_coll').velocity = '0 0 0 0 0 0'
        #     # if (self.state == 0):
        #     #     print "Stop"
        #     #     self.state = 1
        #     self.f.write(str(velocity[0]) + '\n')
        #     self.counter = 0
        #     self.state += 1
        #     if (self.state == 10):
        #         self.f.close()
        #         self.rootNode.getRootContext().animate = False

        # else:
        #     self.counter += 1  
        
        return 0;


def createScene(rootNode):
    rootNode.findData('dt').value = '0.005'
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    mybaseModel = baseModel(rootNode,commandLineArguments)
    return 0;
