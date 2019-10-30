import sys
import math
import Sofa
import socket
import numpy as np
from pathlib import Path
import geometry_util as geo

use_network = False
if use_network:
    import torch
    sys.path.insert(0,'../network/deformable/')
    from model import UNet3D

time_scale = 200.0
# Average from rosbags (time/# messages), 12 and potentially future 13 are calibration(2) bag
all_time_steps = [0.0332, 0.0332, 0.0329, 0.0332, 0.0332, 0.0333, 0.0331, 0.0332, 0.0332, 0.0328, 0.0455, 0.0473] 
data_file = 11
folder_name = 'data' + str(data_file)
#folder_name = 'calibration'

class MeshEnv (Sofa.PythonScriptController):
    robot_step = 0
    partial_step = float(time_scale) # Start from full - partial step ranges from 1 to time_scale
    write_step = 0
    step = 0.05
    axis_scale=100

    # If using network
    if use_network:
        in_channels = 3
        out_channels = 3
        network_path = Path('../network/deformable/checkpoints/models/model_105.pt')
        device = torch.device('cuda')
    
    # Set so the first position is at centre of the platform
    def __init__(self, node, commandLineArguments) : 
        self.count = 0
        self.commandLineArguments = commandLineArguments
        print("Command line arguments for python : "+str(commandLineArguments))
#        self.robot_pos = np.genfromtxt('../dataset/test/' + 'data_cartesian_processed.csv', delimiter=',')
        self.robot_pos = np.genfromtxt('../dataset/2019-10-09-GelPhantom1/dvrk/' + folder_name  + '_robot_cartesian_processed_interpolated.csv', delimiter=',')
        self.createGraph(node)
        self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
        self.grid_order = np.loadtxt('grid_order.txt').astype(int)
        
    def output(self):
        return

    def createGraph(self,rootNode):
        self.rootNode = rootNode
        rootNode.createObject('RequiredPlugin', pluginName='SofaMiscCollision SofaPython')# SofaOpenglVisual')# SofaCUDA')
        rootNode.createObject('VisualStyle', displayFlags='showBehaviorModels hideVisualModels showCollisionModels')# showCollisionModels')# showInteractionForceFields showForceFields')
        rootNode.createObject('FreeMotionAnimationLoop', solveVelocityConstraintFirst=0)
        rootNode.createObject('LCPConstraintSolver', maxIt=1000, tolerance=1e-9, mu=0.9)
        rootNode.createObject('DefaultPipeline', depth=5, verbose=0, draw=0)  
        rootNode.createObject('BruteForceDetection')
        rootNode.createObject('MinProximityIntersection', contactDistance=0.5, alarmDistance=1)
        rootNode.createObject('DefaultContactManager', name='Response', response='FrictionContact')

        # rootNode/phantom
        Phantom = rootNode.createChild('Phantom')
#        scale=[22.9, 35.8, 39.3]
#        translation=[-34.35/22.9, -17.9/35.8, -19.65/39.3]
        Phantom.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Phantom.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
#        Phantom.createObject('MeshGmshLoader', name='loader', filename='meshes/test1.msh')
        Phantom.createObject('RegularGrid', name='grid', n='20 10 11', min='-34.35 -17.9 -18.65', max='34.35 17.9 18.65')
        Phantom.createObject('HexahedronSetTopologyContainer', name='container', src='@grid')
        Phantom.createObject('HexahedronSetGeometryAlgorithms', template='Vec3d')
        Phantom.createObject('MechanicalObject', name='mecha', template='Vec3d')
        Phantom.createObject('HexahedronFEMForceField', youngModulus='5e3', poissonRatio='0.44')
        Phantom.createObject('UniformMass', totalMass=104.1)
        Phantom.createObject('UncoupledConstraintCorrection')

# This is for SimpleBeamTetra_fine.msh
#        Phantom.createObject('FixedConstraint', indices=[12, 107, 27, 115, 13, 203, 45, 211, 14, 285, 293, 15, 96, 94, 109, 114, 113, 210, 209, 273, 292, 291, 23, 95, 22, 104, 25, 192, 41, 200, 43, 274, 55, 282, 57, 88, 90, 91, 92, 105, 187, 188, 189, 201, 269, 270, 271, 283, 0, 66, 18, 77, 1, 164, 37, 175, 2, 246, 51, 257, 3, 191, 205, 287, 59])


# For sofa-created mesh        
        Phantom.createObject('FixedConstraint', indices=[
                                                         2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009, 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                                                         1800, 1801, 1802, 1803, 1804, 1805, 1806, 1807, 1808, 1809, 1810, 1811, 1812, 1813, 1814, 1815, 1816, 1817, 1818, 1819,
                                                         1600, 1601, 1602, 1603, 1604, 1605, 1606, 1607, 1608, 1609, 1610, 1611, 1612, 1613, 1614, 1615, 1616, 1617, 1618, 1619,
                                                         1400, 1401, 1402, 1403, 1404, 1405, 1406, 1407, 1408, 1409, 1410, 1411, 1412, 1413, 1414, 1415, 1416, 1417, 1418, 1419,
                                                         1200, 1201, 1202, 1203, 1204, 1205, 1206, 1207, 1208, 1209, 1210, 1211, 1212, 1213, 1214, 1215, 1216, 1217, 1218, 1219,
                                                         1000, 1001, 1002, 1003, 1004, 1005, 1006, 1007, 1008, 1009, 1010, 1011, 1012, 1013, 1014, 1015, 1016, 1017, 1018, 1019,
                                                         800, 801, 802, 803, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 817, 818, 819,
                                                         600, 601, 602, 603, 604, 605, 606, 607, 608, 609, 610, 611, 612, 613, 614, 615, 616, 617, 618, 619,
                                                         400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 416, 417, 418, 419,
                                                         200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
                                                         0, 1, 2, 3, 4,5,6,7,8,9,10,11,12,13,14,15,16,17,18, 19])
        
        # Collision Node
        CollNode = Phantom.createChild('Coll')
        CollNode.createObject('RegularGridTopology', name='loader', n='20 10 11', min='-34.35 -17.9 -18.65', max='34.35 17.9 18.65')
        CollNode.createObject('MechanicalObject', name='coll', template='Vec3d')
        CollNode.createObject('TTriangleModel')
        CollNode.createObject('TLineModel')
        CollNode.createObject('TPointModel')
        CollNode.createObject('IdentityMapping', input='@../mecha', output='@coll')
        self.Phantom = Phantom

        
        # rootNode/Instrument
        Instrument = rootNode.createChild('Instrument')
        Instrument.createObject('EulerImplicitSolver', printLog='false', rayleighStiffness='0.1', name='odesolver', rayleighMass='0.1')
        Instrument.createObject('CGLinearSolver', threshold='1e-9', tolerance='1e-9', name='linearSolver', iterations='25')
#        Instrument.createObject('SphereGridTopology', nx=5, ny=5, nz=5, radius='3.5')
        Instrument.createObject('MeshObjLoader', name='loader', filename='mesh/sphere.obj')
        Instrument.createObject('MechanicalObject', name='mecha', template='Rigid3d')
        Instrument.createObject('UniformMass', totalMass=1e3)
        Instrument.createObject('UncoupledConstraintCorrection')

        # Collision Node
        CollNode = Instrument.createChild('Coll')
        CollNode.createObject('SphereGridTopology', nx=5, ny=5, nz=5, radius='3.5')
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
        if use_network:
            self.net = UNet3D(in_channels=self.in_channels, out_channels=self.out_channels)
            # Load previous model if requested
            if self.network_path.exists():
                state = torch.load(str(self.network_path))
                self.net.load_state_dict(state['model'])
                self.net = self.net.to(self.device)
                print('Restored model')
            else:
                print('Failed to restore model')
                exit()
            self.net.eval()
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
        # if self.partial_step == time_scale:
        #     pos = np.array(self.Phantom.getObject('mecha').position)
        #     pos = pos[self.grid_order]
        #     np.savetxt(folder_name + "/position" + '%04d' % (self.robot_step) + ".txt",pos)
        # if use_network:
        #     difference = self.robot_pos[self.robot_step,1:8] - self.robot_pos[self.robot_step-1,1:8]
        #     robot_pos = torch.tensor(float(self.partial_step-1/time_scale) * difference + self.robot_pos[self.robot_step,1:8]).to(self.device)
        #     mesh_pos = np.array(self.Phantom.getObject('mecha').position)
        #     robot_pos = robot_pos.unsqueeze(0).float()
        #     pos = torch.tensor(mesh_pos.reshape(25, 9, 9, 3)).to(self.device).float()
        #     pos = pos.permute(3, 0, 1, 2).unsqueeze(0)
        #     network_pos = self.net(torch.tensor(robot_pos), torch.tensor(pos)).squeeze().detach().cpu().numpy()
        #     network_top = network_pos[:,:,-1,:].reshape(3,-1).transpose()
        #     grid_top = self.grid_order.reshape(25,9,9)[:,-1,:]
        #     grid_top = grid_top.reshape(-1)
        #     mesh_pos[grid_top] = network_top
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
            if (self.partial_step < time_scale):
                difference = self.robot_pos[self.robot_step+1,1:8] - self.robot_pos[self.robot_step,1:8]
                update = float(self.partial_step/time_scale) * difference + self.robot_pos[self.robot_step,1:8]
                self.Instrument.getObject('mecha').position = geo.arrToStr(update)
                self.partial_step += 1
            else:
                self.robot_step += 1
                self.Instrument.getObject('mecha').position = geo.arrToStr(self.robot_pos[self.robot_step,1:8])
                self.partial_step = 1
        else:
            self.rootNode.getRootContext().animate = False
        return 0

def createScene(rootNode):
    np.set_printoptions(threshold=sys.maxsize)
    rootNode.findData('dt').value = all_time_steps[data_file]/time_scale
    rootNode.findData('gravity').value = '0 0 0'
    try : 
        sys.argv[0]
    except :
        commandLineArguments = []
    else :
        commandLineArguments = sys.argv
    my_env = MeshEnv(rootNode,commandLineArguments)
    
    return 0
