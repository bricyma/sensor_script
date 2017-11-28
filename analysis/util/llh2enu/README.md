# Documentation

## plot.py 

Visualize the path under different llh2enu transformation methods.
### usage:
    
    ```
    cd sensor_script/analysis/util
    sh ./init.sh 
    python plot.py test.bag 
    ```
    
    The default rosbag path is /home/zhibei/workspace/rosbag/

## llh2enu_gps_transformer.py 

### usage

    ```
    from llh2enu_gps_transformer import gps_transformer
    g = gps_transformer()
    x, y = g.llh2enu_1(lat, lon, h, lat0, lon0, h0)
    ```

### API
* **llh2enu_1**
    
    method referenced http://digext6.defence.gov.au/dspace/bitstream/1947/3538/1/DSTO-TN-0432.pdf 

   	**Input**:
   	lat: latitude to be converted 
   	lon: longitude to be converted 
   	altitude: altitude to be converted 
   	lat0: base latitude 
   	lon0: base longtitude 
   	h0: base altitude
   	
   	**Output:**
   	de: east coordinate 
   	dn: north coordinate
   	
* **llh2enu_2**
    
    method referecenced wikipedia
   	
   	**Input**:
   	lat: latitude to be converted 
   	lon: longitude to be converted 
   	altitude: altitude to be converted 
   	lat0: base latitude 
   	lon0: base longtitude 
   	h0: base altitude
   	
   	**Output:**
   	x: east coordinate 
   	y: north coordinate
   	
* **llh2enu_3**
    
    kitti method
    
   	**Input:**
   	lat_: latitude to be converted 
   	lon_: longitude to be converted 
   	h:
   	baseLat_: base latitude 
   	baseLon_: base longtitude
   	
   	**Output:**
   	x: east coordinate 
   	y: north coordinate
 
   
* **llh2enu_5**
    
    octopus method
    
  	**Input:**
   	lat_: latitude to be converted 
   	lon_: longitude to be converted 
   	h:
   	baseLat_: base latitude 
   	baseLon_: base longtitude
   	
   	**Output:**
   	x: east coordinate 
   	y: north coordinate
   


