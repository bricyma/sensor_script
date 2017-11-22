from vsimple import *
from color import colors
import hashlib

class Box:

    def __init__(self, left=-1, right=-1, top=-1, bottom=-1, type="other", object_id=-1, trajectory_id=-1, info=""):
        self.left = left
        self.right = right
        self.top = top
        self.bottom = bottom
        self.object_id = object_id
        self.trajectory_id = trajectory_id
        self.info = info
        self.type = type

    def parse_json(self, j):
        if 'left' in j:
            self.left = j['left']
        if 'right' in j:
            self.right = j['right']
        if 'top' in j:
            self.top = j['top']
        if 'bottom' in j:
            self.bottom = j['bottom']
        if 'object_id' in j:
            self.object_id = j['object_id']
        if 'oid' in j:
            self.object_id = j['oid']
        if 'trajectory_id' in j:
            self.trajectory_id = j['trajectory_id']
        if 'tid' in j:
            self.object_id = j['tid']
        if 'info' in j:
            self.info = j['info']
        else:
            self.info = str(j)
        if 'type' in j:
            self.type = j['type']

    def to_json(self):
        return {
            "left": self.left,
            "right": self.right,
            "top": self.top,
            "bottom": self.bottom,
            "oid": self.object_id,
            "tid": self.trajectory_id,
            "info": self.info,
            "type": self.type
        }


def imshow_2dbbox(vsimple, id, frame, info, bboxs, tags=None, obj_tags=None):
    """

    :param frame: 2darray
    :param info: info of this bbox
    :param bboxs: list of bbox instances
    :return:
    """
    if tags is None:
        tags = []
    if obj_tags is None:
        obj_tags = []
    c = Canvas(id, info)
    layers = {}
    brushes = {}
    cc = {}
    i = 0
    for b in bboxs:
        if not isinstance(b, Box):
            raise Exception("bboxs must be a list of Box instance")
        bbox = b.to_json()
        layer_name = bbox['type']
        if layer_name not in layers:
            print layer_name
            l = DrawLayer(layer_name)
            c.append_layer(l)
            b = l.get_brush()
            layers[layer_name] = l
            brushes[layer_name] = b
            cc[layer_name] = colors[int(hashlib.md5(layer_name).hexdigest(), 16) % len(colors)]
            i += 1
        if 'tid' in bbox and bbox['tid'] != -1:
            color = colors[bbox['tid'] % len(colors)]
        else:
            color = cc[layer_name]
        brushes[layer_name].add_bbox(left=bbox['left'], top=bbox['top'], right=bbox['right'], bottom=bbox['bottom'],
                                       color=color, info=bbox['info'], oid=bbox['oid'], tid=bbox['tid'])

    encoded_string = vsimple.compress2b64(frame)
    image = Image(encoded_string)
    for layer_name in brushes:
        image.append_brush(brushes[layer_name])
    c.append_image(image)

    for tag in tags:
        c.append_tag(tag)

    for tag in obj_tags:
        c.append_objTag(tag)

    vsimple.imshow(c.get_json())

