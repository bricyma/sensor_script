# Vsimple Brush json

> this is the description of the json format of brush the server generates
> frontend also use this json to draw on canvas

the highest level of the json file contains base64 image, layers to show, tags to mark, and some info of these imges.  Sample:
```
{
	"name": string, (unique id)
	"images": [
		{} (ref: image)
	],
	"tags": [
		{} (ref: tag)
	],
	"obj_tags": [
		{} (ref: tag)
	],
	"layers":[ (draw layer according to this order)
		{
			"name": "layer1",
			"type": "draw"
		},
		{
			"name": "layer2",
			"type": "image"
		},
	],
	"info": ""
}
```

The image json:
```
{
	"b64": "",
	"info": ""
	"layers": {
		"layer1": [], 
		"layer2": {} (ref: layer)
	}
}
```
The draw layer json:
```
[
	｛
		"type": "bbox",
		"left": ,
		"top": ,
		"right": ,
		"bottom": ,
		"color": ,
		"info": ""
	｝,
	｛
		"type": "point",
		"x": ,
		"y": ,
		"color": ,
		"info": ""
	｝
	｛
		"type": "line",
		"points": [{"x": ,"y": }, ...],
		"color": ,
		"info": ""
	｝
]
```
The image layer json:
```
{
	"b64": "",
	"alpha":
}
```
The tag json, like the input element in HTML:
```
[
	{
		"type": "radio",
		"name": "",
		"values": ["", ""]
	},
	{
		"type": "checkbox",
		"name": "",
		"values": ["", ""]
	},
	{
		"type": "text",
		"name": ""
	}
]
```
