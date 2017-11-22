import cv2

from vsimple import *

from os import walk

f = []
for (dirpath, dirnames, filenames) in walk("/mnt/scratch/pengfei/to_kai/segment_sample/images"):
    f.extend(filenames)
    break

vsimple = Vsimple()
for file in f:
    img = cv2.imread("/mnt/scratch/pengfei/to_kai/segment_sample/images/" + file)
    cover = cv2.imread("/mnt/scratch/pengfei/to_kai/segment_sample/results/" + file)
    canvas = Canvas(file, "/mnt/scratch/pengfei/to_kai/segment_sample/images/" + file)
    img_string = vsimple.compress2b64(img)
    image = Image(img_string)
    cover_layer = ImageLayer("segmentation")
    canvas.append_layer(cover_layer)

    imageBrush = cover_layer.get_brush()
    cover_string = vsimple.compress2b64(cover)
    imageBrush.set_image(cover_string, alpha=0.5)
    image.append_brush(imageBrush)

    canvas.append_image(image)

    image2 = Image(img_string)
    cover_layer2 = ImageLayer("segmentation2")
    canvas.append_layer(cover_layer2)
    imageBrush2 = cover_layer2.get_brush()
    imageBrush2.set_image(cover_string, alpha=0.75)
    image2.append_brush(imageBrush2)
    image3 = Image(cover_string)
    canvas.append_image(image2)
    canvas.append_image(image3)
    vsimple.imshow(canvas.get_json())

vsimple.close()