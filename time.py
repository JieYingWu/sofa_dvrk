from pathlib import Path
import sys
import math
import Sofa
import time
import socket
import numpy as np
from pathlib import Path
import geometry_util as geo

# Average from rosbags (time/# messages), 12 and potentially future 13 are calibration(2) bag
all_time_steps = [0.0332, 0.0332, 0.0329, 0.0332, 0.0332, 0.0333, 0.0331, 0.0332, 0.0332, 0.0328, 0.0455, 0.0473]

class MeshEnv (Sofa.PythonScriptController):
    robot_step = 0
    write_step = 0
    write_index = 0

    # Set so the first position is at centre of the platform
    def __init__(self, node, commandLineArguments) :


        print("Command line arguments for python : "+str(commandLineArguments))
        data_file = int(commandLineArguments[1])
        self.time_scale = int(all_time_steps[data_file]*10000)
        print(self.time_scale)
        self.partial_step = self.time_scale
        self.folder_name = 'data' + str(data_file)
        try:
            Path(self.folder_name).mkdir(mode=0o777, parents=False)
        except OSError:
            print("Model path exists")
        #folder_name = 'calibration'

#        self.robot_pos = np.genfromtxt('../dataset/test/' + 'data_cartesian_processed.csv', delimiter=',')
        self.robot_pos = np.genfromtxt('../dataset/2019-10-09-GelPhantom1/dvrk/' + self.folder_name  + '_robot_cartesian_processed_interpolated.csv', delimiter=',')
        self.createGraph(node)
        self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
        self.start_time = time.time()
        
    def output(self):
        return

    def createGraph(self,rootNode):
        self.rootNode = rootNode
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython')# SofaOpenglVisual')# SofaCUDA')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels showCollisionModels hideVisualModels')# showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-9, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)  
        rootNode.createObject('BruteForceDetection')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=1)
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')

        # rootNode/phantom
        Phantom = rootNode.createChild('Phantom')
        scale=[22.9, 35.8, 39.3]
        translation=[-34.35/scale[0], -17.9/scale[1], -19.65/scale[2]]
        Phantom.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Phantom.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
        Phantom.createObject('MeshGmshLoader', name='loader', filename='meshes/SimpleBeamTetra_fine2.msh', translation=translation)
        Phantom.createObject('TetrahedronSetTopologyContainer', name='container', src='@loader')
        Phantom.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
        Phantom.createObject('TetrahedronSetTopologyModifier')
        Phantom.createObject('TetrahedronSetTopologyAlgorithms')
        Phantom.createObject('MechanicalObject', name='mecha', template='Vec3d', scale3d=scale)
        Phantom.createObject('TetrahedronFEMForceField', youngModulus='5e3', poissonRatio='0.44')
        Phantom.createObject('UniformMass', totalMass=104.1)
        Phantom.createObject('UncoupledConstraintCorrection')

# This is for SimpleBeamTetra_fine.msh
#        Phantom.createObject('FixedConstraint', indices=[12, 107, 27, 115, 13, 203, 45, 211, 14, 285, 293, 15, 96, 94, 109, 114, 113, 210, 209, 273, 292, 291, 23, 95, 22, 104, 25, 192, 41, 200, 43, 274, 55, 282, 57, 88, 90, 91, 92, 105, 187, 188, 189, 201, 269, 270, 271, 283, 0, 66, 18, 77, 1, 164, 37, 175, 2, 246, 51, 257, 3, 191, 205, 287, 59])

# This is for SimpleBeamTetra_fine2.msh
        Phantom.createObject('FixedConstraint', indices=[12, 107, 27, 115, 13, 203, 45, 211, 14, 285, 293, 15, 96, 94, 109, 114, 113, 210, 209, 273, 292, 291, 23, 95, 22, 104, 25, 192, 41, 200,                                                    43, 274, 55, 282, 57, 88, 90, 91, 92, 105, 187, 188, 189, 201, 269, 270, 271, 283, 0, 66, 18, 77, 1, 164, 37, 175, 2, 246, 51, 257, 3, 191, 205, 287, 59,
                                                         571, 580, 616, 608, 1171, 1180, 1216, 1208, 1719, 1728, 1764, 1756,
                                                        492, 487, 572, 582, 581, 615, 617, 606, 603, 1089, 1172, 1182, 1181, 1215, 1217, 1206, 1203, 1637, 1720, 1730, 1729, 1763, 1765, 1754, 1751,
                                                        490, 574, 645, 611, 1092, 1174, 1245, 1211, 1640, 1722, 1793, 1759,
                                                        500, 501, 495, 493, 576, 644, 641, 610, 609, 1101, 1096, 1094, 1176, 1244, 1241, 1210, 1209, 1649, 1644, 1642, 1724, 1792, 1789, 1758, 1757,
                                                        499, 494, 558, 561, 1100, 1095, 1158, 1161, 1706, 1709,
                                                        464, 465, 553, 552, 478, 559, 563, 562, 1067, 1153, 1152, 1079, 1159, 1163, 1162, 1615, 1643, 1700, 1627, 1628, 1707, 1711, 1710,
                                                        466, 551, 479, 555, 1068, 1151, 1081, 1155, 1616, 1699, 1629, 1703,
                                                        455, 456, 458, 460, 474, 475, 471, 473, 557, 1059, 1061, 1063, 1076, 1077, 1073, 1075, 1157, 1607, 1609, 1611, 1624, 1625, 1621, 1623, 1705,
                                                        326, 343, 388, 380, 930, 947, 992, 984, 1478, 1495, 1540, 1532,
                                                        477, 1701, 1080, 1648
                                                        ])

        
# This is for SimpleBeamHexa_fine.msh
#        Phantom.createObject('FixedConstraint', indices=[0,3,12,15,68,17,123,121,21,145,13, 210, 38, 227, 14, 292, 52, 309, 308, 50, 267, 270, 49, 246, 2, 188,35, 164,1, 99, 16, 66, 75, 64, 128, 128, 28, 63, 149, 97, 104, 96, 19, 144, 214, 162, 169, 120, 209, 43, 161, 193, 186, 231, 185, 36, 226, 296, 244, 251, 243, 57, 291, 313, 268, 275])
#        Phantom.createObject('MeshExporter', filename='test/mesh', position='@mecha.position', edges='@container.edges', triangles='@container.triangles', tetras='@container.tetras', exportEveryNumberOfSteps=1, format='gmsh')
        
        # # Visual Node
        # VisuNode = Phantom.createChild('Visu_Cyl')
        # VisuNode.createObject('MeshSTLLoader', name='loader', filename='meshes/gel_phantom_1_fine.stl')
        # VisuNode.createObject('OglModel', name='visual', src='@loader', color='yellow')
        # VisuNode.createObject('BarycentricMapping', input='@../mecha', output='@visual')
        
        # Collision Node
        CollNode = Phantom.createChild('Coll')
        CollNode.createObject('MeshTopology', src="@../loader")
        CollNode.createObject('MechanicalObject', src='@../loader', name='coll', scale3d=scale, template='Vec3d')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TPointModel')
        CollNode.createObject('IdentityMapping', input='@../mecha', output='@coll')
        self.Phantom = Phantom

        # rootNode/Instrument
        Instrument = rootNode.createChild('Instrument')
        Instrument.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Instrument.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
#        Instrument.createObject('SphereGridTopology', nx=3, ny=3, nz=3, radius='3')
        Instrument.createObject('MeshObjLoader', name='loader', filename='mesh/sphere.obj')
        Instrument.createObject('MechanicalObject', name='mecha', template='Rigid3d')
        Instrument.createObject('UniformMass', totalMass=1e3)
        Instrument.createObject('UncoupledConstraintCorrection')

        # # Visual Node
        # VisuNode = Instrument.createChild('Visu')
        # VisuNode.createObject('OglModel', name='visual', src='@../loader')
        # VisuNode.createObject('RigidMapping', input='@../mecha', output='@visual')

        # Collision Node
        CollNode = Instrument.createChild('Coll')
        CollNode.createObject('SphereGridTopology', nx=3, ny=3, nz=3, radius='3')
        CollNode.createObject('MechanicalObject', name='coll', template='Vec3d')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TPointModel')
        CollNode.createObject('RigidMapping', input='@../mecha', output='@coll')

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

        if (self.robot_step < self.robot_pos.shape[0]-1):
            if (self.partial_step < self.time_scale):
                difference = self.robot_pos[self.robot_step+1,1:8] - self.robot_pos[self.robot_step,1:8]
                update = float(self.partial_step/float(self.time_scale)) * difference + self.robot_pos[self.robot_step,1:8]
                self.Instrument.getObject('mecha').position = geo.arrToStr(update)
                self.partial_step += 1
            else:
                self.robot_step += 1
                self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
                self.partial_step = 1
        else:
            end_time = time.time()
            print(self.start_time, end_time)
            print('Simulation took ' + str(end_time-self.start_time) + 's')

            self.rootNode.getRootContext().animate = False
        return 0

# sofa cartesian_robot_soft_tissue.py --argv <run_num>
    
def createScene(rootNode):
    np.set_printoptions(threshold=sys.maxsize)
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    print(commandLineArguments)
    data_file = int(commandLineArguments[1])
    rootNode.findData('dt').value = 0.0001
    rootNode.findData('gravity').value = '0 0 0'
    my_env = MeshEnv(rootNode,commandLineArguments)
    
    return 0
