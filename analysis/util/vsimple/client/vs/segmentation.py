from vsimple import *


def imshow_segmentation(vsimple, id, info, frame, cover, alpha=0.5, tags=None):
    if tags is None:
        tags = []
    canvas = Canvas(id, info)
    frame_string = vsimple.compress2b64(frame)
    cover_string = vsimple.compress2b64(cover)

    image = Image(frame_string)
    cover_layer = ImageLayer("segmentation")
    canvas.append_layer(cover_layer)

    imageBrush = cover_layer.get_brush()
    imageBrush.set_image(cover_string, alpha=alpha)
    image.append_brush(imageBrush)

    canvas.append_image(image)

    vsimple.imshow(canvas.get_json())