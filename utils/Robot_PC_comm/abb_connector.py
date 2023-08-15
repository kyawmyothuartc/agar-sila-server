'''
Source - https://github.com/robotics/open_abb
Michael Dawson-Haggerty

abb.py: contains classes and support functions which interact with an ABB Robot running our software stack (RAPID code module SERVER)


For functions which require targets (XYZ positions with quaternion orientation),
targets can be passed as [[XYZ], [Quats]] OR [XYZ, Quats]

'''

import socket
import json 
import time
import inspect
from threading import Thread
from collections import deque
import logging
import numpy as np

#from TECAN_PIPETTE.serial_interface import Pipette



log = logging.getLogger(__name__)
log.addHandler(logging.NullHandler())
    
class Robot:
    def __init__(self, 
                 ip          = '192.168.1.3',  #localhost = '127.0.0.1' , robotIp = '169.254.173.125'
                 port_motion = 5000,
                 port_logger = 5001):

        self.delay   = .08

        self.connect_motion((ip, port_motion))
        #log_thread = Thread(target = self.get_net, 
        #                    args   = ((ip, port_logger))).start()
        
        self.set_units('millimeters', 'degrees')
        self.set_tool()
        self.set_workobject()
        self.set_speed()
        self.set_zone()

        formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
        handler_stream = logging.StreamHandler()
        handler_stream.setFormatter(formatter)
        handler_stream.setLevel(logging.DEBUG)
        log = logging.getLogger('abb')
        log.setLevel(logging.DEBUG)
        log.addHandler(handler_stream)

    def connect_motion(self, remote):        
        log.info('Attempting to connect to robot motion server at %s', str(remote))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(2.5)
        self.sock.connect(remote)
        self.sock.settimeout(None)
        log.info('Connected to robot motion server at %s', str(remote))

    def connect_logger(self, remote, maxlen=None):
        self.pose   = deque(maxlen=maxlen)
        self.joints = deque(maxlen=maxlen)
        
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(remote)
        s.setblocking(1)
        try:
            while True:
                data = map(float, s.recv(4096).split())
                if   int(data[1]) == 0: 
                    self.pose.append([data[2:5], data[5:]])
                #elif int(data[1]) == 1: self.joints.append([a[2:5], a[5:]])
        finally:
            s.shutdown(socket.SHUT_RDWR)

    def set_units(self, linear, angular):
        units_l = {'millimeters': 1.0,
                   'meters'     : 1000.0,
                   'inches'     : 25.4}
        units_a = {'degrees' : 1.0,
                   'radians' : 57.2957795}
        self.scale_linear = units_l[linear]
        self.scale_angle  = units_a[angular]

    def set_cartesian(self, pose):
        '''
        Executes a move immediately from the current pose,
        to 'pose', with units of millimeters.
        '''
        msg  = "01 " + self.format_pose(pose)   
        return self.send(msg)

    def set_joints(self, joints):
        '''
        Executes a move immediately, from current joint angles,
        to 'joints', in degrees. 
        '''
        if len(joints) != 6: return False
        msg = "02 "
        for joint in joints: msg += format(joint*self.scale_angle, "+08.2f") + " " 
        msg += "#" 
        return self.send(msg)

    def get_cartesian(self):
        '''
        Returns the current pose of the robot, in millimeters
        '''
        msg = "03 #"
        data = self.send(msg).split()
        r = [float(s) for s in data]
        return [r[2:5], r[5:9]]

    def get_joints(self):
        '''
        Returns the current angles of the robots joints, in degrees. 
        '''
        msg = "04 #"
        data = self.send(msg).split()
        return [float(s) / self.scale_angle for s in data[2:8]]

    def get_external_axis(self):
        '''
        If you have an external axis connected to your robot controller
        (such as a FlexLifter 600, google it), this returns the joint angles
        '''
        msg = "05 #"
        data = self.send(msg).split()
        return [float(s) for s in data[2:8]]
       
    def get_robotinfo(self):
        '''
        Returns a robot- unique string, with things such as the
        robot's model number. 
        Example output from and IRB 2400:
        ['24-53243', 'ROBOTWARE_5.12.1021.01', '2400/16 Type B']
        '''
        msg = "98 #"
        data = str(self.send(msg))[5:].split('*')
        log.debug('get_robotinfo result: %s', str(data))
        return data

    def set_tool(self, tool=[[0,0,0], [1,0,0,0]]):
        '''
        Sets the tool centerpoint (TCP) of the robot. 
        When you command a cartesian move, 
        it aligns the TCP frame with the requested frame.
        
        Offsets are from tool0, which is defined at the intersection of the
        tool flange center axis and the flange face.
        '''
        msg       = "06 " + self.format_pose(tool)    
        self.send(msg)
        self.tool = tool

    def load_json_tool(self, file_obj):
        if file_obj.__class__.__name__ == 'str':
            file_obj = open(filename, 'rb');
        tool = check_coordinates(json.load(file_obj))
        self.set_tool(tool)
        
    def get_tool(self): 
        log.debug('get_tool returning: %s', str(self.tool))
        return self.tool

    def set_workobject(self, work_obj=[[0,0,0],[1,0,0,0]]):
        '''
        The workobject is a local coordinate frame you can define on the robot,
        then subsequent cartesian moves will be in this coordinate frame. 
        '''
        msg = "07 " + self.format_pose(work_obj)   
        self.send(msg)

    def set_speed(self, speed=[100,50,50,50]):
        '''
        speed: [robot TCP linear speed (mm/s), TCP orientation speed (deg/s),
                external axis linear, external axis orientation]
        '''

        if len(speed) != 4: return False
        msg = "08 " 
        msg += format(speed[0], "+08.1f") + " " 
        msg += format(speed[1], "+08.2f") + " "  
        msg += format(speed[2], "+08.1f") + " " 
        msg += format(speed[3], "+08.2f") + " #"     
        self.send(msg)

    def set_zone(self, 
                 zone_key     = 'z1', 
                 point_motion = False, 
                 manual_zone  = []):
        zone_dict = {'z0'  : [.3,.3,.03], 
                    'z1'  : [1,1,.1], 
                    'z5'  : [5,8,.8], 
                    'z10' : [10,15,1.5], 
                    'z15' : [15,23,2.3], 
                    'z20' : [20,30,3], 
                    'z30' : [30,45,4.5], 
                    'z50' : [50,75,7.5], 
                    'z100': [100,150,15], 
                    'z200': [200,300,30]}
        '''
        Sets the motion zone of the robot. This can also be thought of as
        the flyby zone, AKA if the robot is going from point A -> B -> C,
        how close do we have to pass by B to get to C
        
        zone_key: uses values from RAPID handbook (stored here in zone_dict)
        with keys 'z*', you should probably use these

        point_motion: go to point exactly, and stop briefly before moving on

        manual_zone = [pzone_tcp, pzone_ori, zone_ori]
        pzone_tcp: mm, radius from goal where robot tool centerpoint 
                   is not rigidly constrained
        pzone_ori: mm, radius from goal where robot tool orientation 
                   is not rigidly constrained
        zone_ori: degrees, zone size for the tool reorientation
        '''

        if point_motion: 
            zone = [0,0,0]
        elif len(manual_zone) == 3: 
            zone = manual_zone
        elif zone_key in zone_dict.keys(): 
            zone = zone_dict[zone_key]
        else: return False
        
        msg = "09 " 
        msg += str(int(point_motion)) + " "
        msg += format(zone[0], "+08.4f") + " " 
        msg += format(zone[1], "+08.4f") + " " 
        msg += format(zone[2], "+08.4f") + " #" 
        self.send(msg)

    def buffer_add(self, pose):
        '''
        Appends single pose to the remote buffer
        Move will execute at current speed (which you can change between buffer_add calls)
        '''
        msg = "30 " + self.format_pose(pose) 
        self.send(msg)

    def buffer_set(self, pose_list):
        '''
        Adds every pose in pose_list to the remote buffer
        '''
        self.clear_buffer()
        for pose in pose_list: 
            self.buffer_add(pose)
        if self.buffer_len() == len(pose_list):
            log.debug('Successfully added %i poses to remote buffer', 
                      len(pose_list))
            return True
        else:
            log.warn('Failed to add poses to remote buffer!')
            self.clear_buffer()
            return False

    def clear_buffer(self):
        msg = "31 #"
        data = self.send(msg)
        if self.buffer_len() != 0:
            log.warn('clear_buffer failed! buffer_len: %i', self.buffer_len())
            raise NameError('clear_buffer failed!')
        return data

    def buffer_len(self):
        '''
        Returns the length (number of poses stored) of the remote buffer
        '''
        msg = "32 #"
        data = self.send(msg).split()
        return int(float(data[2]))

    def buffer_execute(self):
        '''
        Immediately execute linear moves to every pose in the remote buffer.
        '''
        msg = "33 #"
        return self.send(msg)

    def set_external_axis(self, axis_unscaled=[-550,0,0,0,0,0]):
        if len(axis_values) != 6: return False
        msg = "34 "
        for axis in axis_values:
            msg += format(axis, "+08.2f") + " " 
        msg += "#"   
        return self.send(msg)

    def move_circular(self, pose_onarc, pose_end):
        '''
        Executes a movement in a circular path from current position, 
        through pose_onarc, to pose_end
        '''
        msg_0 = "35 " + self.format_pose(pose_onarc)  
        msg_1 = "36 " + self.format_pose(pose_end)

        data = self.send(msg_0).split()
        if data[1] != '1': 
            log.warn('move_circular incorrect response, bailing!')
            return False
        return self.send(msg_1)

    def set_dio(self, value, IOname):
        '''
        A function to set a physical DIO line on the robot.
        For this to work you're going to need to edit the RAPID function
        and fill in the DIO you want this to switch. 
        '''
        if IOname == 'Ring_motor':
            Id = 0
        
        elif IOname == 'Ring_light':
            Id = 1

        msg = '97 ' + str(int(bool(value))) +' '+ str(Id) + ' #'
        print(msg)
        #return 
        return self.send(msg)
             
    def send(self, message, wait_for_response=True):
        '''
        Send a formatted message to the robot socket.
        if wait_for_response, we wait for the response and return it
        '''
        caller = inspect.stack()[1][3]
        log.debug('%-14s sending: %s', caller, message)
        self.sock.send(message.encode('utf-8'))
        time.sleep(self.delay)
        if not wait_for_response: return
        data = self.sock.recv(4096)
        log.debug('%-14s recieved: %s', caller, data)
        return data
        
    def format_pose(self, pose):
        pose = check_coordinates(pose)
        msg  = ''
        for cartesian in pose[0]:
            msg += format(cartesian * self.scale_linear,  "+08.1f") + " " 
        for quaternion in pose[1]:
            msg += format(quaternion, "+08.5f") + " " 
        msg += "#" 
        return msg       
        
    def close(self):
        self.send("99 #", False)
        self.sock.shutdown(socket.SHUT_RDWR)
        self.sock.close()
        log.info('Disconnected from ABB robot.')

    def call_subroutine(self,routine,*args):
        '''
        This is an additional function and modification need to be done on the robot end.
        This function is used to call a defined subroutine in the RAPID code.

        '''
        msg = "11"
        
        if routine == 'tip picking':
            msg += " 101" + " "
            msg += str(args[0]) + " "
            msg += str(args[1]) + " #"
            
            return self.send(msg)
            
            
        elif routine == 'colony picking':
            msg += " 102" + " "
            msg += str(args[0]) + " "
            msg += str(args[1]) + " #"
            return self.send(msg)
           

        elif routine == 'Streaking1':
            msg += " 103" +" "
            msg += str(args[0]) + " "
            msg += str(args[1]) + " #"
            print(msg)
            return self.send(msg)
  
        
        elif routine == 'Streaking2':
            msg += " 104" +" "
            msg += str(args[0]) + " "
            msg += str(args[1]) + " #"
            print(msg)
            return self.send(msg)
        
        elif routine == 'DeepWell':
            msg += " 105" +" "
            msg += str(args[0]) + " "
            msg += str(args[1]) + " #"
            print(msg)
            return self.send(msg)

    def set_tip_counter(self,count):
        msg = '12 ' + str(count) + ' #'
        return self.send(msg)

    def get_tip_counter(self):
        msg = '13 #'
        data = self.send(msg)
        return data
    
    
    # def __enter__(self):
    #     return self
        
    # def __exit__(self, type, value, traceback):
    #     self.close()


def check_coordinates(coordinates):
    if ((len(coordinates) == 2) and
        (len(coordinates[0]) == 3) and 
        (len(coordinates[1]) == 4)): 
        return coordinates
    elif (len(coordinates) == 7):
        return [coordinates[0:3], coordinates[3:7]]
    log.warn('Recieved malformed coordinate: %s', str(coordinates))
    raise NameError('Malformed coordinate!')


def Robot_start(robotControl, Tip_count, colony_coordinates, output_selection, type_selection):
    #first check if robot connected
    try:
        #0. Initialization
        Initialize = robotControl
        option =[]
        tipcounter = Tip_count # this value needs to be maintained at both the GUI level and the Robot program
        #check for all selection before providing access and contain the selection information into a list.
        
        selection = ['Streaking1','Streaking2','DeepWell'] # example 
    
        #1.Code to 'TURN ON' RingLight:
        
        #2.Code to trigger camera and obtain the pose of the colony for all 8 cells: for example [(0,pos0),(1,pos1),...] or just a list of poses
        #All positions mentioned are Examples, the real values will be generated by the pose determined by the vision algorithm.
        index = colony_coordinates
        
        #3.Code to 'TURN OFF' RingLight
    
        #4.Code to swing open the RingLight Assembly
        
        #5. Obtain the Global variable values: a. Tipcount and deep well row-count. (assuming that the same deepwell will be used (need more information from IMCB)).
        # These values can be maintained in the GUI code??. Need to decide if deepwell count needs to be maintained.
        option = 'Streaking1'
        
        #6.Enable the robot motion code based on the selection and all other considerations
        
        if output_selection == 'Streaking Pattern 1' and type_selection == '1 to 1':
            #processA1_Streaking_patter1(Initialize, tipcounter)
            typ_selec = 0
            for x,y  in index:
                if y != "Null":
                    processA1_Streaking_pattern1_1to1(Initialize,tipcounter,x,y, typ_selec)
                    #dummy_process(Initialize,tipcounter,x,y, typ_selec)
                    tipcounter = tipcounter + 1

            Initialize.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])
            print(tipcounter)
            
        elif output_selection == 'Streaking Pattern 1' and type_selection == '1 to Many':
            
            typ_selec = 1
            z=[]  # list used to identify if single value given or multiple values
            for x,y in index:
                if y != "Null":
                    z.append(x)
                    q=y 

            print(z)        
            if len(z) != 1:           
                pass # Exception
                
            else:
                
                processA1_Streaking_pattern1_1tomany(Initialize,tipcounter,z[0],q,typ_selec)
                tipcounter = tipcounter + 1

                Initialize.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])
        
        elif output_selection == 'Streaking Pattern 2' and type_selection == '1 to 1':
            typ_selec = 0
            for x,y  in index:
                if y != "Null":
                    processA2_Streaking_pattern2_1to1(Initialize,tipcounter,x,y, typ_selec)
                    tipcounter = tipcounter + 1
            
            Initialize.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])
        
        elif output_selection == 'Streaking Pattern 2' and type_selection == '1 to Many':
            typ_selec = 1
            z=[]  # list used to identify if single value given or multiple values
            for x,y in index:
                if y != "Null":
                    z.append(x)
                    q=y 

            print(z)        
            if len(z) != 1:           
                pass # Exception
                
            else:
                
                processA2_Streaking_pattern2_1tomany(Initialize,tipcounter,z[0],q,typ_selec)
                tipcounter = tipcounter + 1

                Initialize.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])

        elif output_selection == 'Deep Well' and type_selection == '1 to 1':
            typ_selec = 0
            for x,y  in index:
                if y != "Null":
                    processA3_deep_well_1to1(Initialize,tipcounter,x,y, typ_selec)
                    tipcounter = tipcounter + 1

            Initialize.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])
            print(tipcounter)

        elif output_selection == 'Deep Well' and type_selection == '1 to Many':
            typ_selec = 1
            z=[]  # list used to identify if single value given or multiple values
            for x,y in index:
                if y != "Null":
                    z.append(x)
                    q=y 

            print(z)        
            if len(z) != 1:           
                pass # Exception
                
            else:
                
                processA3_deep_well_1tomany(Initialize,tipcounter,z[0],q,typ_selec)
                tipcounter = tipcounter + 1

                Initialize.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])

        #7.Update the information of the data tracing and also update the tip counter on the GUI program/Robot Program.
        
          
        return tipcounter

    except socket.error as msg:
    
        if msg=='timed out': return 'SocketError'
            
def Robot_stop():
    pass

def global_tip_counter(robotControl, request,value = 0):
    # to get the Global Tip counter data
    # request = 0 is getting tip count from Robot controller
    # request = 1 is setting tip count value in Robot controller
    try:
        
        if request == 0:
            Value = robotControl
            Global_Tip_count = Value.get_tip_counter()
            lastString = str(Global_Tip_count).split()
            lastString = lastString[-1]
            lastString = lastString.replace("'",'')
            
            # Value.close()
            return int(lastString)

        else:
            Value = robotControl
            Value.set_tip_counter(value)
            # Value.close()

    except socket.error as msg:
    
        if msg=='timed out': return 'SocketError'

#can use decorator to elimimate duplication of codes for types of transfer and streaking. Also can conisder decorator for eliminating tip position estimation
def processA1_Streaking_pattern1_1to1(Proc_init, tip_count,cell_position, coordinates,type_selec):
    counter = tip_count
    tip_array = np.arange(96).reshape(12,8)
    
    #identify the row and cloumn position of the tip to be picked
    tip_position = np.where(tip_array == counter)
    offsetx =''.join(str(tip_position[0]).replace('[','')) 
    offsetx = offsetx.replace(']','')
    offsety =''.join(str(tip_position[1]).replace('[','')) 
    offsety = offsety.replace(']','')
    

    Proc_init.call_subroutine('tip picking',offsetx,offsety)
    Proc_init.call_subroutine('colony picking',cell_position,0)
    # use the colony_co-ordinates to move to the exact location using the value
    
    print(coordinates)
    Proc_init.set_cartesian(coordinates)
    time.sleep(2)

    Proc_init.call_subroutine('colony picking',cell_position,1)
    Proc_init.call_subroutine('Streaking1', cell_position,type_selec)

    pipette = Pipette("COM3")
    pipette.Eject()
    
    #Proc_init.close()

def processA1_Streaking_pattern1_1tomany(Proc_init, tip_count,cell_position, coordinates,type_selec):
    counter = tip_count
    tip_array = np.arange(96).reshape(12,8)
    
    #identify the row and cloumn position of the tip to be picked
    tip_position = np.where(tip_array == counter)
    offsetx =''.join(str(tip_position[0]).replace('[','')) 
    offsetx = offsetx.replace(']','')
    offsety =''.join(str(tip_position[1]).replace('[','')) 
    offsety = offsety.replace(']','')
    

    Proc_init.call_subroutine('tip picking',offsetx,offsety)
    Proc_init.call_subroutine('colony picking',cell_position,0)
    # use the colony_co-ordinates to move to the exact location using the value
    
    print(coordinates)
    Proc_init.set_cartesian(coordinates)
    time.sleep(2)

    Proc_init.call_subroutine('colony picking',cell_position,1)

    for cellno in range(7):
        Proc_init.call_subroutine('Streaking1', cellno, type_selec)
    
    Proc_init.call_subroutine('Streaking1', 7, 0) # this is the last cell of the output agar plate, hence passing fixed values


    pipette = Pipette("COM3")
    pipette.Eject()

    #Proc_init.close()

def processA2_Streaking_pattern2_1to1(Proc_init, tip_count,cell_position, coordinates,type_selec):
    counter = tip_count
    tip_array = np.arange(96).reshape(12,8)
    
    #identify the row and cloumn position of the tip to be picked
    tip_position = np.where(tip_array == counter)
    offsetx =''.join(str(tip_position[0]).replace('[','')) 
    offsetx = offsetx.replace(']','')
    offsety =''.join(str(tip_position[1]).replace('[','')) 
    offsety = offsety.replace(']','')
    

    Proc_init.call_subroutine('tip picking',offsetx,offsety)
    Proc_init.call_subroutine('colony picking',cell_position,0)
    # use the colony_co-ordinates to move to the exact location using the value
    
    print(coordinates)
    Proc_init.set_cartesian(coordinates)
    time.sleep(2)

    Proc_init.call_subroutine('colony picking',cell_position,1)
    Proc_init.call_subroutine('Streaking2', cell_position,type_selec)

    pipette = Pipette("COM3")
    pipette.Eject()

    #Proc_init.close()

def processA2_Streaking_pattern2_1tomany(Proc_init, tip_count,cell_position, coordinates,type_selec):
    counter = tip_count
    tip_array = np.arange(96).reshape(12,8)
    
    #identify the row and cloumn position of the tip to be picked
    tip_position = np.where(tip_array == counter)
    offsetx =''.join(str(tip_position[0]).replace('[','')) 
    offsetx = offsetx.replace(']','')
    offsety =''.join(str(tip_position[1]).replace('[','')) 
    offsety = offsety.replace(']','')
    

    Proc_init.call_subroutine('tip picking',offsetx,offsety)
    Proc_init.call_subroutine('colony picking',cell_position,0)
    # use the colony_co-ordinates to move to the exact location using the value
    
    print(coordinates)
    Proc_init.set_cartesian(coordinates)
    time.sleep(2)

    Proc_init.call_subroutine('colony picking',cell_position,1)

    for cellno in range(7):
        Proc_init.call_subroutine('Streaking2', cellno, type_selec)
    
    Proc_init.call_subroutine('Streaking2', 7, 0) # this is the last cell of the output agar plate, hence passing fixed values


    pipette = Pipette("COM3")
    pipette.Eject()

    #Proc_init.close()
    
def processA3_deep_well_1to1(Proc_init, tip_count,cell_position, coordinates,type_selec):
    counter = tip_count
    tip_array = np.arange(96).reshape(12,8)
    
    #identify the row and cloumn position of the tip to be picked
    tip_position = np.where(tip_array == counter)
    offsetx =''.join(str(tip_position[0]).replace('[','')) 
    offsetx = offsetx.replace(']','')
    offsety =''.join(str(tip_position[1]).replace('[','')) 
    offsety = offsety.replace(']','')
    

    Proc_init.call_subroutine('tip picking',offsetx,offsety)
    Proc_init.call_subroutine('colony picking',cell_position,0)
    # use the colony_co-ordinates to move to the exact location using the value
    
    print(coordinates)
    Proc_init.set_cartesian(coordinates)
    time.sleep(2)

    Proc_init.call_subroutine('colony picking',cell_position,1)
    Proc_init.call_subroutine('DeepWell', cell_position,type_selec)

    pipette = Pipette("COM3")
    pipette.Eject()

    #Proc_init.close()

def processA3_deep_well_1tomany(Proc_init, tip_count,cell_position, coordinates,type_selec):
    counter = tip_count
    tip_array = np.arange(96).reshape(12,8)
    
    #identify the row and cloumn position of the tip to be picked
    tip_position = np.where(tip_array == counter)
    offsetx =''.join(str(tip_position[0]).replace('[','')) 
    offsetx = offsetx.replace(']','')
    offsety =''.join(str(tip_position[1]).replace('[','')) 
    offsety = offsety.replace(']','')
    

    Proc_init.call_subroutine('tip picking',offsetx,offsety)
    Proc_init.call_subroutine('colony picking',cell_position,0)
    # use the colony_co-ordinates to move to the exact location using the value
    
    print(coordinates)
    Proc_init.set_cartesian(coordinates)
    time.sleep(2)

    Proc_init.call_subroutine('colony picking',cell_position,1)

    for cellno in range(7):
        Proc_init.call_subroutine('DeepWell', cellno, type_selec)
    
    Proc_init.call_subroutine('DeepWell', 7, 0) # this is the last cell of the output agar plate, hence passing fixed values


    pipette = Pipette("COM3")
    pipette.Eject()

    #Proc_init.close()

def processB_extraction():
    pass    


#######################################################
# test decorator function for process functions /Implement later
# def decorator_process(original_func):
#     def wrapper(*args,**kwargs):
        
#         counter = args[1]
#         tip_array = np.arange(96).reshape(12,8)
        
#         #identify the row and cloumn position of the tip to be picked
#         tip_position = np.where(tip_array == counter)
#         offsetx =''.join(str(tip_position[0]).replace('[','')) 
#         offsetx = offsetx.replace(']','')
#         offsety =''.join(str(tip_position[1]).replace('[','')) 
#         offsety = offsety.replace(']','')

#         args[0].call_subroutine('tip picking',offsetx,offsety)
        
#         test = original_func(*args,**kwargs)
#         return test
#     return wrapper

# @decorator_process
# def dummy_process(Proc_init, tip_count,cell_position, coordinates,type_selec):
    
    # Proc_init.call_subroutine('colony picking',cell_position,0)
    # # use the colony_co-ordinates to move to the exact location using the value
    
    # print(coordinates)
    # Proc_init.set_cartesian(coordinates)
    # time.sleep(2)

    # Proc_init.call_subroutine('colony picking',cell_position,1)
    # Proc_init.call_subroutine('Streaking1', cell_position,type_selec)

    # pipette = Pipette("COM3")
    # pipette.Eject()
    
    # # return to Home position
    # Proc_init.set_cartesian([[310.81, -4.60, 450.67], [0.006, -0.003, 1.000, -0.001]])

    # Proc_init.close()

# Comment this entire program entry section when running application using GUI  
if __name__ == '__main__':
    formatter = logging.Formatter("[%(asctime)s] %(levelname)-7s (%(filename)s:%(lineno)3s) %(message)s", "%Y-%m-%d %H:%M:%S")
    handler_stream = logging.StreamHandler()
    handler_stream.setFormatter(formatter)
    handler_stream.setLevel(logging.DEBUG)
    log = logging.getLogger('abb')
    log.setLevel(logging.DEBUG)
    log.addHandler(handler_stream)
    #Robot_start()

    pose0 =[[450.77, -228.15, 26.66], [0.007, 0.007, -1.000, 0.006]]
    # #pose0 =[[452.56, 211.25, 8.04],[0.001, -0.009, 1.000, -0.016]]
    testin = Robot()
    #testin.set_cartesian(pose0)
    testin.get_cartesian()
    #testin.call_subroutine('Streaking2',7,0)
    #testin.set_dio(False,'Ring_light')
    #testin.set_dio(False,'Ring_motor')
    testin.call_subroutine('tip picking',0,0)
    testin.call_subroutine('colony picking',1,0)
    #testin.call_subroutine('colony picking',3,1)
    
   #codes yet to be implemented
#1. Keep a looping connection with server -> not needed as every command will establish a connection when called. 
#2. A open and close command for calling Robot function (link to START/STOP buttons)
#3. All Program logic for sequence (including all subprocesses)
#4. Logger/ acknowlegment return can be used to display on the GUI.
#5. Display variables such as a.tip count after process, b.colony identification success, c. timestamp,    

