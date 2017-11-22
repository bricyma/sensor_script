import cv2
from vsimple import *
import json
from vs import Line, imshow_line


with open('/mnt/scratch/siyuanliu/offiline-aggregation/agg_res/20161231-line4-stage4-aggregated_result.json') as fp:
    vsimple = Vsimple()
    for string in fp.readlines():
        aggregated_result = json.loads(string.strip())
        raw_file = aggregated_result['raw_file']
        frame = cv2.imread(raw_file)
        lines = []
        for line in aggregated_result['aggregated_result']:
            try:
                info = str(line[1]) + ' , ' + str(line[2])
                for l in line[0]:
                    temp = Line(data=l, color='green', info=info)
                    lines.append(temp)
            except (IndexError, ValueError) as e:
                print e
                continue
        imshow_line(vsimple, raw_file, frame, raw_file, lines)
    vsimple.close()
