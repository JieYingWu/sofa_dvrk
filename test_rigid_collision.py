"""
baseModelPython
is based on the scene 

but it uses the SofaPython plugin. 
Further informations on the usage of the plugin can be found in 
sofa/applications/plugins/SofaPython/doc/SofaPython.pdf
To launch the scene, type 
runSofa .py --argv 123
The sofa python plugin might have to be added in the sofa plugin manager, 
i.e. add the sofa python plugin in runSofa->Edit->PluginManager.
The arguments given after --argv can be used by accessing self.commandLineArguments, e.g. combined with ast.literal_eval to convert a string to a number.

The current file has been written by the python script
'{' is not recognized as an internal or external command,
operable program or batch file.\scn2python.py
Author of '{' is not recognized as an internal or external command,
operable program or batch file.\scn2python.py: Christoph PAULUS, christoph.paulus@inria.fr
"""

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
        rootNode.createObject('RequiredPlugin', pluginName='CImgPlugin')
        rootNode.createObject('VisualStyle', displayFlags='showBehavior showCollisionModels')
        rootNode.createObject('BruteForceDetection', name='N2')
        rootNode.createObject('CollisionPipeline', draw='0', depth='5', verbose='0')
        rootNode.createObject('LCPConstraintSolver', mu='0.000000001', maxIt='1000', printLog='0', initial_guess='false', build_lcp='false', tolerance='1e-3')
        rootNode.createObject('DefaultContactManager')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst='1')
        rootNode.createObject('CollisionResponse', mu='0', response='FrictionlessContact', name='collision response')
        # rootNode.createObject('MinProximityIntersection', contactDistance='0.5', alarmDistance='1', name='Proximity')
        rootNode.createObject('LocalMinDistance', contactDistance='0.5', alarmDistance='1')
        rootNode.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='cg_odesolver', rayleighMass='0.1')
        rootNode.createObject('CGLinearSolver', threshold='1.0e-9', tolerance='1.0e-9', name='linear solver', iterations='100')
        # rootNode.createObject('DiscreteIntersection')

        # rootNode/arm_1
        arm_1 = rootNode.createChild('arm_1')
        self.arm_1 = arm_1
        arm_1.createObject('MechanicalObject', name='Arm_1', template='Rigid', velocity='0 -1 0 0 0 0', translation='0 5 100')
        arm_1.createObject('UniformMass', totalMass='1')
        # arm_1.createObject('FixedRotationConstraint',template='Rigid', FixedXRotation='0', FixedYRotation='0', FixedZRotation='0')
        # arm_1.createObject('UncoupledConstraintCorrection')
        
        # Collision Node of arm_1
        CollNode1 = arm_1.createChild('coll_arm_1')
        CollNode1.createObject('MeshObjLoader', name='loader', filename='meshes/arm.obj')
        CollNode1.createObject('Mesh', src='@loader')
        CollNode1.createObject('MechanicalObject', name='CollModel')
        CollNode1.createObject('Triangle')
        CollNode1.createObject('TLineModel')
        CollNode1.createObject('TPointModel')
        CollNode1.createObject('RigidMapping')

        # rootNode/arm_2
        arm_2 = rootNode.createChild('arm_2')
        self.arm_2 = arm_2
        arm_2.createObject('MechanicalObject', name='Arm_2', template='Rigid', velocity='0 1 0 0 0 0', translation='0 -45 100')
        arm_2.createObject('UniformMass', totalMass='1')
        # arm_1.createObject('FixedRotationConstraint',template='Rigid', FixedXRotation='0', FixedYRotation='0', FixedZRotation='0')
        # arm_1.createObject('UncoupledConstraintCorrection')
        
        # Collision Node of arm_2
        CollNode2 = arm_2.createChild('coll_arm_2')
        CollNode2.createObject('MeshObjLoader', name='loader', filename='meshes/arm.obj')
        CollNode2.createObject('Mesh', src='@loader')
        CollNode2.createObject('MechanicalObject', name='CollModel')
        CollNode2.createObject('Triangle')
        CollNode2.createObject('TLineModel')
        CollNode2.createObject('TPointModel')
        CollNode2.createObject('RigidMapping')            

        # rootNode/Center
        Center = rootNode.createChild('Center')
        self.Center = Center
        Center.createObject('MeshSTLLoader', name='loader', filename='meshes/center_large.stl')
        Center.createObject('Mesh', src='@loader')
        Center.createObject('MechanicalObject', name='Center', template='Vec3d')
        Center.createObject('UniformMass', totalMass='10')
        # Center.createObject('BoxROI', name='Box', box='99 -50 -1  101 50 101', drawBoxes='1')
        # Center.createObject('FixedConstraint', indices='@Box.indices')
        Center.createObject('TriangularFEMForceField', youngModulus='100', poissonRatio='0.1', method='large')
        Center.createObject('MeshSpringForceField', stiffness='100', damping='1')
        # Center.createObject('RestShapeSpringsForceField', name='Springs', stiffness='0.1')
        # Center.createObject('StiffSpringForceField', stiffness='100')
        Center.createObject('UncoupledConstraintCorrection')

        # Collision Node of center
        CollCenter = Center.createChild('CollCenter')
        CollCenter.createObject('MeshSTLLoader', name='loader', filename='meshes/center_large.stl')
        CollCenter.createObject('Mesh', src='@loader')
        CollCenter.createObject('MechanicalObject', name='coll_center')
        CollCenter.createObject('TTriangleModel', selfCollision='true')
        CollCenter.createObject('TPointModel', selfCollision='true')
        CollCenter.createObject('TLineModel', selfCollision='true')
        CollCenter.createObject('TSphereModel', selfCollision='true')
        CollCenter.createObject('IdentityMapping')
        # CollCenter.createObject('BarycentricMapping')

        return 0;

    def onMouseButtonLeft(self, mouseX,mouseY,isPressed):
        ## usage e.g.
        #if isPressed : 
        #    print "Control+Left mouse button pressed at position "+str(mouseX)+", "+str(mouseY)
        return 0;

    def onKeyReleased(self, c):
        ## usage e.g.
        if c=="E" :
            self.arm_1.getObject('Arm_1').velocity = '0 0 0 0 0 0'
            self.arm_2.getObject('Arm_2').velocity = '0 0 0 0 0 0'
        if c=="A" :
            self.arm_1.getObject('Arm_1').velocity = '0 -1 0 0 0 0'
            self.arm_2.getObject('Arm_2').velocity = '0 -1 0 0 0 0'

        return 0;

    def initGraph(self, node):
        return 0;

    def onKeyPressed(self, c):
        ## usage e.g.
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
        simtime = self.rootNode.time
        if (self.rootNode.time > 7.7 and self.rootNode.time < 8):
            self.arm_1.getObject('Arm_1').velocity = '0 0 0 0 0 0'
            self.arm_2.getObject('Arm_2').velocity = '0 0 0 0 0 0'
        if (self.rootNode.time > 8):
            self.arm_1.getObject('Arm_1').velocity = '0 -1 0 0 0 0'
            self.arm_2.getObject('Arm_2').velocity = '0 -1 0 0 0 0'

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
