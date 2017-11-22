import base64
import time
import requests
from threading import Lock, Thread, ThreadError
import socket
from SimpleXMLRPCServer import SimpleXMLRPCServer


class Vsimple:

    def __init__(self, width=480, height=360, frame_rate=15):
        self.lock = Lock()
        self.continue_play = False
        self.gap = 1.0 / frame_rate
        self.quit = False
        self.thread = Thread(target=self.rpc_thread)
        self.thread.start()
        self.tags = []
        self.resize_width = width
        self.resize_height = height

    def __del__(self):
        self.close()

    def close(self):
        self.quit = True
        print "Vsimple Closing..."
        print "If not close. Please click next or play in the webpage"
        self.thread.join()

    def compress2b64(self, frame):
        import cv2
        small = cv2.resize(frame, (self.resize_width, self.resize_height))
        cnt = cv2.imencode('.jpg', small)[1]
        b64 = base64.encodestring(cnt)
        return b64

    def imshow(self, data):
        self.lock.acquire()
        r = requests.post("http://localhost:25001/imshow", json=data)
        if self.continue_play:
            self.lock.release()
        # print self.continue_play
        time.sleep(self.gap)

    # def imshow_frame(self, frame, data):
    #     b64 = self.compress2b64(frame)
    #     self.imshow_b64(b64, data)
    #
    # def imshow_b64(self, b64, data):
    #     self.imshow({'b64': b64, 'json': data})

    def pause_play(self):
        # print "play"
        self.continue_play = (self.continue_play is False)
        self.next_image()
        return True

    def next_image(self):
        # print "next"
        try:
            self.lock.release()
        except ThreadError as e:
            print e
            return False
        return True

    def tag(self, message):
        self.tags.append(message)
        return True

    def rpc_thread(self):
        server = SimpleXMLRPCServer(("localhost", 25000))
        print "Listening on port 25000 on ", socket.gethostname()
        server.register_function(self.pause_play, "pause_play")
        server.register_function(self.next_image, "next_image")
        server.register_function(self.tag, "tag")
        while not self.quit:
            server.handle_request()



