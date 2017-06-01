# time sync check for both chronyc tracking and chronyc sources
import os
from datetime import datetime
import time


class TimeSync:
    def __init__(self):
        self.timenow = str(datetime.now())
        self.date = self.timenow[:10]
        os.system('mkdir t' + self.timenow[:10])

    def check(self):
        print self.timenow[:19].replace(' ', '-')
        self.timenow = str(datetime.now())
        os.system('chronyc tracking > ' + 't' + self.date + '/track-' + self.timenow[:19].replace(' ', '-') + '.txt')
        os.system('chronyc sources > ' + 't' + self.date + '/source-' + self.timenow[:19].replace(' ', '-') + '.txt')

    def run(self):
        while 1:
            self.check()
            time.sleep(1800)


if __name__ == '__main__':
    syncRecorder = TimeSync()
    syncRecorder.run()

