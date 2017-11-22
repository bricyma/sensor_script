import json

with open("/mnt/scratch/kaizhou/vsimple_data/test.json") as jfile:
    j = json.load(jfile)

data = {}
for f in j:
    data[f['frame']] = f['feature']['objs']


