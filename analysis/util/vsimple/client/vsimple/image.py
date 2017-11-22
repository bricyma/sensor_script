class Image:

    def __init__(self, b64, info=""):
        self.b64 = b64
        self.info = info
        self.layers = {}

    def append_brush(self, brush):
        self.layers[brush.name] = brush.get_json()

    def get_json(self):
        return {
            "b64": self.b64,
            "info": self.info,
            "layers": self.layers
        }