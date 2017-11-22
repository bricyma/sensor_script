import requests
import eviltransform

class Map:

    def __init__(self, lat=32.869250, lng=-117.227873, zoom=16):
        self.go(lat, lng, zoom)

    def go(self, lat, lng, zoom=16):
        lat, lng = gcj2wgs(lat, lng)
        data = {
            "cmd": "init",
            "data": {
                "lat": lat,
                "lng": lng,
                "zoom": zoom
            }
        }
        self._cmd(data)

    def scatter(self, lat, lng, color='red', size=2):
        lat, lng = gcj2wgs(lat, lng)
        data = {
            "cmd": "scatter",
            "data": {
                "lat": lat,
                "lng": lng,
                "color": color,
                "size": size
            }
        }
        self._cmd(data)
    def scatter_green(self, lat, lng, color='green', size=2):
        lat, lng = gcj2wgs(lat, lng)
        data = {
            "cmd": "scatter",
            "data": {
                "lat": lat,
                "lng": lng,
                "color": color,
                "size": size
            }
        }
        self._cmd(data)
    
    def plot(self, lat, lng, color='red', size=2):
        lat, lng = gcj2wgs(lat, lng)
        data = {
            "cmd": "plot",
            "data": {
                "lat": lat,
                "lng": lng,
                "color": color,
                "size": size
            }
        }
        self._cmd(data)

    def clear(self):
        data = {
            "cmd": "clear",
            "data": {}
        }
        self._cmd(data)

    def _cmd(self, data):
        r = requests.post("http://localhost:25001/mapshow", json=data)


def gcj2wgs(lat, lng):
    ret = eviltransform.wgs2gcj(lat, lng)
    return ret[0], ret[1]