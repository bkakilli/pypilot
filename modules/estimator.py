import socket, struct
#from bluetooth import *

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
    def getPoses(self, i):
        return [0,0,0,0,0,0]

class TangoPoseEstimator(EstimatorBase):
    def getPoses(self, i):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def join(self):
        raise NotImplementedError

    def getPoses(self, i):
        raise NotImplementedError

class ViconTrackerEstimator(EstimatorBase):

    class ViconObject:
        item_id = 0
        item_datasize = 0
        item_name = ''
        pose = [0,0,0,0,0,0]

    poscapturethread = None
    poses = []
    def __init__(self, ip, port, drone_name):

        self.udpsock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        self.udpsock.bind((ip, port))
        self.drone_name = drone_name

    def run(self):
        self.poscapturethread = threading.Thread(target=self.positionReceiver)
        self.poscapturethread.start()

    def stop(self):
        self.stopReceiver = True
        self.join()

    def join(self):
        if self.poscapturethread:
            self.poscapturethread.join()

    def getPoses(self, i=-1):
        if i>-1:
            return poses[i]
        return poses

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

            for v_obj in v_objects:
                poses.append(v_obj.pose)
                if v_obj.item_name == self.drone_name:
                    poses.insert(0, v_obj.pose)
                else:
                    poses.append(v_obj.pose)
