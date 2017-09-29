import socket, threading, math
from bluetooth import *
from struct import *

class EstimatorBase:
    def run(self):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def join(self):
        raise NotImplementedError

    def getPoses(self, i):
        raise NotImplementedError

class ZeroEstimator(EstimatorBase):
    # This class is to support manual control or use of external
    # position estimate schemes. It basically provides 'always zero'
    # pose. An appropriate actuator should be used.
    def __init__(self, cfg, logger):
        pass
    def run(self):
        return True

    def stop(self):
        pass

    def join(self):
        pass

    def getPoses(self, i=-1):
        return [[0,0,0,0,0,0]]

class TangoPoseEstimator(EstimatorBase):

    cfg = None
    poscapturethread = None
    poses = []

    def __init__(self, cfg, logger):
        self.cfg = cfg
        self.logger = logger
        self.btUUID = cfg['btUUID']
        self.btAddr = cfg['btAddr']
        self.stopReceiver = False

    def getPoses(self, i):
        raise NotImplementedError
    
    def run(self):
        # BT Connection
        service_matches = find_service( uuid = self.btUUID, address = self.btAddr )

        if len(service_matches) == 0:
            self.logger.error("Couldn't find the Bluetooth device service.")
            self.stopReceiver = True
            return False

        first_match = service_matches[0]
        port = first_match["port"]
        name = first_match["name"]
        host = first_match["host"]

        self.logger.info("Connecting to Bluetooth device \"%s\" on %s" % (name, host))

        # Create the client socket
        try:
            self.sock=BluetoothSocket( RFCOMM )
            self.sock.connect((host, port))

            self.logger.info("Connected to bluetooth pose estimator.")
        
            self.poscapturethread = threading.Thread(target=self.positionReceiver)
            self.poscapturethread.start()
            
        except Exception as error:
            self.logger.error('Could not create or connect to bluetooth socket:')
            self.logger.error(repr(error))
            self.stopReceiver = True
            return False
        
        return True

    def stop(self):
        self.stopReceiver = True
        self.join()

    def join(self):
        if self.poscapturethread:
            self.poscapturethread.join()

    def getPoses(self):
        return self.poses
        
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    def Quaternion_toEulerianAngle(self, x, y, z, w):
        ysqr = y*y
        
        t0 = +2.0 * (w * x + y*z)
        t1 = +1.0 - 2.0 * (x*x + ysqr)
        X = math.degrees(math.atan2(t0, t1))
        
        t2 = +2.0 * (w*y - z*x)
        t2 =  1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        Y = math.degrees(math.asin(t2))
        
        t3 = +2.0 * (w * z + x*y)
        t4 = +1.0 - 2.0 * (ysqr + z*z)
        Z = math.degrees(math.atan2(t3, t4))
        
        return [X,Y,Z] 
        
    def positionReceiver(self):
        while not self.stopReceiver:
            received = self.sock.recv(28)
            rPose = unpack('<fffffff', received)
            rot = self.Quaternion_toEulerianAngle(rPose[3],rPose[4],rPose[5],rPose[6])
            rPose = [rPose[0],rPose[1],rPose[2],rot[0],rot[1],rot[2]]
            
            self.poses = []
            self.poses.append(rPose)
        
        self.sock.close()
        

class ViconTrackerEstimator(EstimatorBase):

    class ViconObject:
        item_id = 0
        item_datasize = 0
        item_name = ''
        pose = [0,0,0,0,0,0]

    cfg = None
    poscapturethread = None
    poses = []

    def __init__(self, cfg, logger):
        self.cfg = cfg
        self.logger = logger
        self.stopReceiver = False

    def run(self):
        
        try:
            self.udpsock = socket.socket(socket.AF_INET, # Internet
                                 socket.SOCK_DGRAM) # UDP
            self.udpsock.bind((cfg['UDP_IP'], cfg['UDP_PORT']))
        except:
            self.logger.error('Could not connect to Vicon.')
            self.stopReceiver = True
            return False
            
        self.drone_name = cfg['VICON_DRONENAME']
        
        self.poscapturethread = threading.Thread(target=self.positionReceiver)
        self.poscapturethread.start()
        
        return True

    def stop(self):
        self.stopReceiver = True
        self.join()
        self.logger('Estimator shut down.')

    def join(self):
        if self.poscapturethread:
            self.poscapturethread.join()

    def getPoses(self, i=-1):
        if i>-1:
            return self.poses[i]
        return self.poses

    def viconObjectDeserialize(self, data):
        frameNumber =struct.unpack('i',data[:4])[0]
        itemsInBlock = struct.unpack('B',data[5])[0]
        offset = 5
        objects = []
        for i in range(itemsInBlock):
            obj = ViconObject()
            # obj.item_id = struct.unpack('B',data[offset])[0]
            # obj.item_datasize = struct.unpack('H',data[offset+1:offset+3])[0]
            obj.item_name = data[offset+3:offset+27]
            obj.item_name = obj.item_name.replace('\0','')
            obj.pose = [
            struct.unpack('d',data[offset+27:offset+35])[0]/1000,
            struct.unpack('d',data[offset+35:offset+43])[0]/1000,
            struct.unpack('d',data[offset+43:offset+51])[0]/1000,
            struct.unpack('d',data[offset+51:offset+59])[0],
            struct.unpack('d',data[offset+59:offset+67])[0],
            struct.unpack('d',data[offset+67:offset+75])[0]
            ]

            objects.append(obj)

            offset = offset + 75

    def positionReceiver(self):
        while not self.stopReceiver:
            data, addr = self.udpsock.recvfrom(256) # buffer size is 1024 byt$
            v_objects = viconObjectDeserialize(data)
            
            self.poses = []
            for v_obj in v_objects:
                self.poses.append(v_obj.pose)
                if v_obj.item_name == self.drone_name:
                    self.poses.insert(0, v_obj.pose)
                else:
                    self.poses.append(v_obj.pose)
