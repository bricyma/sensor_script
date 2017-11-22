import cv2
from vsimple import *
import json

# read the objectdetecion result from json
with open("/mnt/scratch/kaizhou/vsimple_data/test.json") as jfile:
    j = json.load(jfile)

data = {}
for f in j:
    data[f['frame']] = f['feature']['objs']

# use opencv to handle video
cap = cv2.VideoCapture('/mnt/scratch/kaizhou/vsimple_data/test.mp4')
frame_num = 0

num_of_frames = int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))
vsimple = Vsimple()
for frame_id in xrange(num_of_frames):
    print frame_id
    ret, frame = cap.read()

    canvas = Canvas(str(frame_id), "This is test info : " + str(frame_id))
    bicycle_layer = DrawLayer("bicycle")
    canvas.append_layer(bicycle_layer)
    bicycle_brush = bicycle_layer.get_brush()
    car_layer = DrawLayer("car")
    canvas.append_layer(car_layer)
    car_brush = car_layer.get_brush()
    ped_layer = DrawLayer("ped")
    canvas.append_layer(ped_layer)
    ped_brush = ped_layer.get_brush()


    try:
        for obj in data[frame_id]:
            if obj['type'] == 'BICYCLE':
                bicycle_brush.add_bbox(left=obj['left'], top=obj['top'], right=obj['right'], bottom=obj['bottom'],
                                       color='blue', info="confidence: " + str(obj['confidence']))
            elif obj['type'] == 'CAR':
                car_brush.add_bbox(left=obj['left'], top=obj['top'], right=obj['right'], bottom=obj['bottom'],
                                   color='red', info="confidence: " + str(obj['confidence']))
            elif obj['type'] == 'PEDESTRIAN':
                ped_brush.add_bbox(left=obj['left'], top=obj['top'], right=obj['right'], bottom=obj['bottom'],
                                   color='green', info="confidence: " + str(obj['confidence']))
    except KeyError as e:
        print e
    encoded_string = vsimple.compress2b64(frame)
    image = Image(encoded_string)
    image.append_brush(bicycle_brush)
    image.append_brush(car_brush)
    image.append_brush(ped_brush)
    canvas.append_image(image)

    radio_tag = RadioTag('rate', ['good', 'bad'])
    canvas.append_tag(radio_tag)
    checkbox_tag = CheckboxTag('elements', ['car', 'bicycle', 'building'])
    canvas.append_tag(checkbox_tag)
    text_tag = TextTag('reason')
    canvas.append_tag(text_tag)

    # image2 = Image(encoded_string)
    # canvas.append_image(image2)

    vsimple.imshow(canvas.get_json())
cap.release()
vsimple.close()
print vsimple.tags