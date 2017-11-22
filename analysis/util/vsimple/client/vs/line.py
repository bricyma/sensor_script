from vsimple import *


class Line:

    def __init__(self, data=None, color="red", info=""):
        self.data = []
        self.info = info
        self.color = color
        if data is not None:
            self.parse_raw(data=data)

    def parse_raw(self, data):
        self.data = []
        for i in range(0, len(data), 2):
            self.data.append({
                'x': float(data[i]),
                'y': float(data[i + 1])
            })

    def add_point(self, x, y):
        self.data.append({
            'x': x,
            'y': y
        })

    def parse_json(self, j):
        if 'points' in j and isinstance(j['points'], list):
            self.data = []
            for po in j['points']:
                if 'x' in po and 'y' in po:
                    self.add_point(po['x'], po['y'])
        if 'info' in j:
            self.info = j['info']

    def to_json(self):
        return {
            'points': self.data,
            'info': self.info
        }


def imshow_line(vsimple, id, frame, info, lines, tags=None):
    if tags is None:
        tags = []
    canvas = Canvas(str(id), info)
    draw_layer = DrawLayer('line')
    canvas.append_layer(draw_layer)
    brush = draw_layer.get_brush()
    for line in lines:
        if not isinstance(line, Line):
            raise Exception("lines must be a list of Line instance")
        brush.add_line(line.data, color=line.color, info=line.info)
    encoded_string = vsimple.compress2b64(frame)
    image = Image(encoded_string)
    image.append_brush(brush)
    canvas.append_image(image)

    for tag in tags:
        canvas.append_tag(tag)

    vsimple.imshow(canvas.get_json())
