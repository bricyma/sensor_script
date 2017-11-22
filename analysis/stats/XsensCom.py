import matplotlib.pyplot as plt
import rosbag
import sys
import numpy as np
from numpy import array, cov, corrcoef


Scale_Factor = 0.05 / float(2**15)
Toinspvax = '/novatel_data/inspvax'
Tocorrimudata = '/novatel_data/corrimudata'
Toraw = '/novatel_data/rawimu'
ToXsensdata = '/xsens_driver/imu_data_str' 

class Orientation:
    def __init__(self):
        bag_path = './'
        bagname = bag_path + sys.argv[1]

        self.bag = rosbag.Bag(bagname)

        self.Yaw_Xsens1 = []
        self.Pitch_Xsens1 = []
        self.Roll_Xsens1 = []
        
        self.accX_Xsens1 = []
        self.accY_Xsens1 = []
        self.accZ_Xsens1 = []

        self.gyrZ_Xsens1 = []
        self.gyrY_Xsens1 = []
        self.gyrX_Xsens1 = []

        self.Yaw_Xsens2 = []
        self.Pitch_Xsens2 = []
        self.Roll_Xsens2 = []
        
        self.accX_Xsens2 = []
        self.accY_Xsens2 = []
        self.accZ_Xsens2 = []

        self.gyrZ_Xsens2 = []
        self.gyrY_Xsens2 = []
        self.gyrX_Xsens2 = []

        ##novatel
        self.Yaw_Novatel = []
        self.Pitch_Novatel = []
        self.Roll_Novatel = []
        
        self.accX_Novatel = []
        self.accY_Novatel = []
        self.accZ_Novatel = []

        self.gyrZ_Novatel = []
        self.gyrY_Novatel = []
        self.gyrX_Novatel = []

    def readbag(self):
        #input topic_name corrimudata
        for topic, msg, t in self.bag.read_messages(topics=[Toinspvax]):

            self.Yaw_Novatel.append(msg.azimuth)
            self.Pitch_Novatel.append(msg.pitch)
            self.Roll_Novatel.append(msg.roll)
        for topic, msg, t in self.bag.read_messages(topics=[Tocorrimudata]):

            self.accX_Novatel.append(msg.x_accel*200)
            self.accY_Novatel.append(msg.y_accel*200)
            self.accZ_Novatel.append(msg.z_accel*200)

            self.gyrY_Novatel.append(msg.roll_rate*200)
            self.gyrX_Novatel.append(msg.pitch_rate*200)
            self.gyrZ_Novatel.append(msg.yaw_rate*200)
        for topic, msg, t in self.bag.read_messages(topics=[ToXsensdata]):
        	# orientation
            str_data = msg.data
            #Yaw
            yaw_index = str_data.index("'Yaw':")
            yaw_end = str_data.find(',',yaw_index)
            yaw_data = str_data[yaw_index+6:yaw_end] 
            yaw_data = float(yaw_data)
            if yaw_data < 0:
            	yaw_data += 360
            self.Yaw_Xsens1.append(360-yaw_data)
            #Pitch
            pitch_index = str_data.index("'Pitch':")
            pitch_end = str_data.find(',',pitch_index)
            pitch_data = str_data[pitch_index+8:pitch_end]
            pitch_data = float(pitch_data)
            
            #Roll
            roll_index = str_data.index("'Roll':")
            roll_end = str_data.find('}',roll_index)
            roll_data = str_data[roll_index+7:roll_end]
            roll_data = float(roll_data)
            self.Roll_Xsens1.append(-roll_data)###??????????????????????
            self.Pitch_Xsens1.append(-pitch_data)###??????????????????????????????
            #Xsens_Yaw,novatel_yaw

            # acceleration
            accX_index = str_data.index("'accX':")
            accX_end = str_data.find(',',accX_index)
            accX_data = float(str_data[accX_index+7:accX_end])
            accY_index = str_data.index("'accY':")
            accY_end = str_data.find(',',accY_index)
            accY_data = float(str_data[accY_index+7:accY_end])
            accZ_index = str_data.index("'accZ':")
            accZ_end = str_data.find('}',accZ_index)
            accZ_data = (str_data[accZ_index+7:accZ_end])
            accZ_data = float(accZ_data)-9.81
            self.accX_Xsens1.append(accX_data)
            self.accY_Xsens1.append(accY_data)
            self.accZ_Xsens1.append(accZ_data)

            gyrY_index = str_data.index("'gyrY':")
            gyrY_end = str_data.find('}',gyrY_index)
            gyrY_data = float(str_data[gyrY_index+7:gyrY_end])
            gyrX_index = str_data.index("'gyrX':")
            gyrX_end = str_data.find(',',gyrX_index)
            gyrX_data = float(str_data[gyrX_index+7:gyrX_end])
            gyrZ_index = str_data.index("'gyrZ':")
            gyrZ_end = str_data.find(',',gyrZ_index)
            gyrZ_data = float(str_data[gyrZ_index+7:gyrZ_end])
            self.gyrX_Xsens1.append(gyrX_data)
            self.gyrY_Xsens1.append(gyrY_data)
            self.gyrZ_Xsens1.append(gyrZ_data)

       


        self.bag.close()
    # def plotYaw(self):
    #     p1 = plt.subplot(311)
    #     p1.plot(self.Yaw_Xsens1, 'r', label='Xsens mti-30 Yaw')
    #     p1.legend(loc='upper left')
    #     p1.plot(self.Yaw_Xsens2, 'b', label='Xsens 710 Yaw')
    #     p1.legend(loc='upper left')
    #     p2 = plt.subplot(312)
    #     p2.plot(self.Pitch_Xsens1,'r',label='Xsens mti-30 Pitch')
    #     p2.legend(loc='upper left')
    #     p2.plot(self.Pitch_Xsens2,'b',label='Xsens 710 Pitch')
    #     p3 = plt.subplot(313)
    #     p3.plot(self.Roll_Xsens1,'r',label='Xsens mti-30 Roll')
    #     p3.legend(loc='upper left')
    #     p3.plot(self.Roll_Xsens2,'b',label='Xsens 710 Roll')
    #     plt.show()

    # def plotAcc(self):
    #     p1 = plt.subplot(311)
    #     p1.plot(self.accX_Xsens1, 'r', label='Xsens mti-30 acc X')
    #     p1.legend(loc='upper left')
    #     p1.plot(self.accX_Xsens2, 'b', label='Xsens 710 acc X')
    #     p1.legend(loc='upper left')
    #     p2 = plt.subplot(312)
    #     p2.plot(self.accY_Xsens1,'r',label='Xsens mti-30 acc Y')
    #     p2.plot(self.accY_Xsens2,'b',label='Xsens 710 acc Y')
    #     p2.legend(loc='upper left')
    #     p3 = plt.subplot(313)
    #     p3.plot(self.accZ_Xsens1,'r',label='Xsens mti-30 acc Z')
    #     p3.legend(loc='upper left')
    #     p3.plot(self.accZ_Xsens2,'b',label='Xsens 710 acc Z')
    #     plt.show()

    def plotXsensNovatel(self):
        p1 = plt.subplot(311)
        p1.plot(self.Yaw_Novatel, 'r', label='Novatel Yaw')
        p1.legend(loc='upper left')
        p1.plot(self.Yaw_Xsens1, 'b', label='Xsens Yaw')
        p1.legend(loc='upper left')
        p2 = plt.subplot(312)
        p2.plot(self.Pitch_Novatel,'r',label='Novatel Pitch')
        p2.legend(loc='upper left')
        p2.plot(self.Pitch_Xsens1,'b',label='Xsens Pitch')
        p3 = plt.subplot(313)
        p3.plot(self.Roll_Novatel,'r',label='Novatel Roll')
        p3.legend(loc='upper left')
        p3.plot(self.Roll_Xsens1,'b',label='Xsens Roll')
        plt.show()

if __name__ == '__main__':
    ori = Orientation()
    ori.readbag()
    #ori.calOri()
    ori.plotXsensNovatel()
    # ori.plotYaw()
    # ori.plotAcc()