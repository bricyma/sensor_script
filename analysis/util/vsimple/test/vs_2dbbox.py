from vsimple import *
from vs import imshow_2dbbox, Box
import cv2
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

    radio_tag = RadioTag('rate', ['good', 'bad'])
    checkbox_tag = CheckboxTag('elements', ['car', 'bicycle', 'building'])
    text_tag = TextTag('reason')

    bboxs = []
    if frame_id in data:
        for b in data[frame_id]:
            bbox = Box()
            bbox.parse_json(b)
            bbox.info = "confidence: " + str(b['confidence'])
            bbox.trajectory_id = b['id']
            bboxs.append(bbox)

    imshow_2dbbox(vsimple=vsimple, id=frame_id, frame=frame, info="test info" + str(frame_id), bboxs=bboxs, tags=[checkbox_tag, text_tag], obj_tags=[radio_tag])
    print vsimple.tags

cap.release()
vsimple.close()
print vsimple.tags

