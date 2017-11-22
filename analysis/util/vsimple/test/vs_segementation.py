import cv2
from vsimple import *
from os import walk
from vs import imshow_segmentation

f = []
for (dirpath, dirnames, filenames) in walk("/mnt/scratch/pengfei/to_kai/segment_sample/images"):
    f.extend(filenames)
    break

vsimple = Vsimple()
for file in f:
    img = cv2.imread("/mnt/scratch/pengfei/to_kai/segment_sample/images/" + file)
    cover = cv2.imread("/mnt/scratch/pengfei/to_kai/segment_sample/results/" + file)
    imshow_segmentation(vsimple, file, "/mnt/scratch/pengfei/to_kai/segment_sample/images/" + file, img, cover)

vsimple.close()