from vsimple.map import Map
import ujson as json
import time

gps_list = []
e = 0.0000001
with open('/mnt/scratch/sync_sd/car_record/map/20161123/gps_imu_dat/gps_imu_recoder.txt') as f:
    for line in f.readlines():
        try:
            data = json.loads(line.strip())
        except Exception as e:
            continue
        if 'latitude' in data and 'longitude' in data:
            if len(gps_list) > 0 and abs(gps_list[-1]['lat'] - data['latitude']) < e and  abs(gps_list[-1]['lng'] - data['longitude']) < e:
                continue
            gps_list.append({
                'lat': data['latitude'],
                'lng': data['longitude'],
            })

print len(gps_list)

m = Map()
sliced_gps = gps_list[:10000:50]
length = len(sliced_gps)
for i, point in enumerate(sliced_gps):

    m.plot(point['lat'], point['lng'])
    # m.scatter(point['lat'], point['lng'])

    # recenter map every 4 points
    if i == 0 or i % 4 == 0 or i == length - 1:
        m.go(point['lat'], point['lng'], 19 - 1.0 * i/length * 9)
    time.sleep(0.5)

time.sleep(10)
m.clear()