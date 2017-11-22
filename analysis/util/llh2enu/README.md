**llh2enu_gps_transformer.py Documentation**
----
  This python file ontains 4 APIs which convert llh value to enu value using different conversion ways.

* **llh2enu_1**

   	**Input:**
   	lat: latitude to be converted <br>
   	lon: longitude to be converted <br>
   	altitude: altitude to be converted <br>
   	lat0: base latitude <br>
   	lon0: base longtitude <br>
   	h0: base altitude
   	
   	**Output:**
   	de: east coordinate <br>
   	dn: north coordinate
   	
* **llh2enu_2**

   	**Input:**
   	lat: latitude to be converted <br>
   	lon: longitude to be converted <br>
   	altitude: altitude to be converted <br>
   	lat0: base latitude <br>
   	lon0: base longtitude <br>
   	h0: base altitude
   	
   	**Output:**
   	x: east coordinate <br>
   	y: north coordinate
   	
* **llh2enu_3**

   	**Input:**
   	lat_: latitude to be converted <br>
   	lon_: longitude to be converted <br>
   	baseLat_: base latitude <br>
   	baseLon_: base longtitude
   	
   	**Output:**
   	x: east coordinate <br>
   	y: north coordinate
   
* **llh2enu_4 (llh2ecef + ecef2enu)**

	**Input:**
   	lat: latitude to be converted <br>
   	lon: longitude to be converted <br>
   	h: altitude to be converted <br>
   	lat0: base latitude <br>
   	lon0: base longtitude <br>
   	h0: base altitude
   	
   	**Output:**
   	de: east coordinate <br>
   	dn: north coordinate <br>
   	du: up coordinate
   	

**Sample Call:**
-------------
Solution 1:
  ```
  de, dn = llh2enu_1(lat, lon, h, lat0, lon0, h0)
  ```
  
Solution 2:
  ```
  x, y = llh2enu_2(lat, lon, h, lat0, lon0, h0)
  ```
  
Solution 3:
  ```
  x, y = llh2enu_3(lat_, lon_, baseLat_, baseLon_)
  ```
  
Solution 4:
  ```
 de, dn, du = llh2enu_4(lat, lon, h, lat0, lon0, h0)
  ```