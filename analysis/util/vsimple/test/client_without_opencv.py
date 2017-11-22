import base64

from vsimple import *
from trajectory_extractor import data

vsimple = Vsimple()
with open("data/test.jpg", "rb") as image_file:
    canvas = Canvas("test", "This is test info")
    bicycle_layer = DrawLayer("bicycle")
    canvas.append_layer(bicycle_layer)
    bicycle_brush = bicycle_layer.get_brush()
    car_layer = DrawLayer("car")
    canvas.append_layer(car_layer)
    car_brush = car_layer.get_brush()
    ped_layer = DrawLayer("ped")
    canvas.append_layer(ped_layer)
    ped_brush = ped_layer.get_brush()

    for obj in data[0]:
        if obj['type'] == 'BICYCLE':
            bicycle_brush.add_bbox(left=obj['left'], top=obj['top'], right=obj['right'], bottom=obj['bottom'],
                                   color='blue', info="confidence: " + str(obj['confidence']))
        elif obj['type'] == 'CAR':
            car_brush.add_bbox(left=obj['left'], top=obj['top'], right=obj['right'], bottom=obj['bottom'],
                               color='red', info="confidence: " + str(obj['confidence']))
        elif obj['type'] == 'PEDESTRIAN':
            ped_brush.add_bbox(left=obj['left'], top=obj['top'], right=obj['right'], bottom=obj['bottom'],
                               color='green', info="confidence: " + str(obj['confidence']))
    encoded_string = base64.b64encode(image_file.read())
    image = Image(encoded_string)
    image.append_brush(bicycle_brush)
    image.append_brush(car_brush)
    image.append_brush(ped_brush)
    canvas.append_image(image)

    radio_tag = RadioTag('rate', ['good', 'bad'])
    canvas.append_tag(radio_tag)

    print canvas.get_json()

    vsimple.imshow(canvas.get_json())
vsimple.close()
print vsimple.tags
