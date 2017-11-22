# Vsimple Map Json

The hightest level of json file.

```
{
    "cmd": "", // "init", "scatter", "plot", "clear"
    "data": {}
}
```

## init
```
{
    "lat": float,
    "lng": float,
    "zoom": float
}
```

## scatter
```
{
    "name": string,
    "lat": float,
    "lng": float,
    "color": string,
    "size": float,
    "info": string
}
```

## plot
```
{
    "name": string,
    "lat": float,
    "lng": float,
    "color": string,
    "size": float,
    "info": string
}
```

## clear
```
{}
```