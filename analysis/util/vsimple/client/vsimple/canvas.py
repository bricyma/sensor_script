class Canvas:
    """
    Show how to draw the shapes or filter on the image
    """
    def __init__(self, name, info=""):
        self.name = name
        self.info = info
        self.images = []
        self.layers = []
        self.tags = []
        self.objTags = []

    def append_layer(self, layer):
        self.layers.append(layer.get_layer())

    def append_image(self, image):
        self.images.append(image.get_json())

    def append_tag(self, tag):
        self.tags.append(tag.get_json())

    def append_objTag(self, tag):
        self.objTags.append(tag.get_json())

    def get_json(self):
        return {
            "name": self.name,
            "images": self.images,
            "tags": self.tags,
            "obj_tags": self.objTags,
            "layers": self.layers,
            "info": self.info
        }
