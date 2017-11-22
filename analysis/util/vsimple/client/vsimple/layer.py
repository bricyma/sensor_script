from abc import ABCMeta, abstractmethod
from brush import DrawBrush, ImageBrush


class Layer:
    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.name = name

    @abstractmethod
    def get_brush(self):
        pass

    @abstractmethod
    def get_layer(self):
        pass


class DrawLayer(Layer):

    def __init__(self, name):
        super(DrawLayer, self).__init__(name)
        self.brush = DrawBrush(name)

    def get_brush(self):
        return self.brush

    def get_layer(self):
        return {
            "name": self.name,
            "type": "draw"
        }


class ImageLayer(Layer):

    def __init__(self, name):
        super(ImageLayer, self).__init__(name)
        self.brush = ImageBrush(name)

    def get_brush(self):
        return self.brush

    def get_layer(self):
        return {
            "name": self.name,
            "type": "image"
        }