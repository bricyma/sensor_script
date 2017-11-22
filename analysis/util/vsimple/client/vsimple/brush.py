from abc import ABCMeta, abstractmethod

class Brush:
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name

    @abstractmethod
    def get_json(self):
        pass


class DrawBrush(Brush):

    def __init__(self, name):
        super(DrawBrush, self).__init__(name)
        self.j = []

    def add_bbox(self, left, top, right, bottom, color="red", info="", oid=-1, tid=-1):
        self.j.append({
            "type": "bbox",
            "left": left,
            "top": top,
            "right": right,
            "bottom": bottom,
            "color": color,
            "info": info,
            "oid": oid,
            "tid": tid
        })

    def add_point(self, x, y, color="red", info=""):
        self.j.append({
            "type": "point",
            "x": x,
            "y": y,
            "color": color,
            "info": info
        })

    def add_line(self, points, color="red", info=""):
        self.j.append({
            "type": "line",
            "points": points,
            "color": color,
            "info": info
        })

    def get_json(self):
        return self.j


class ImageBrush(Brush):

    def __init__(self, name):
        super(ImageBrush, self).__init__(name)
        self.j = {}

    def set_image(self, b64, alpha=1.0):
        self.j['b64'] = b64
        self.j['alpha'] = alpha

    def get_json(self):
        return self.j


