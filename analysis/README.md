# GPS/IMU Test Tool

This is a performance auto-test tool for GPS & IMU.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for testing performance. See usage instructions for detailed explaination.

## Usage Instructions

1. clone this repository to local environment.
```
git clone https://github.com/TuSimple/octopus-gps.git
```
2. set the ros bag to be tested: put the ros bag into testbag directory, rename the file to "test.bag".

3. run analysis script under this folder.
```
sh analysis.sh
```
4. choose if you are testing GPS or IMU

![prompt1](https://github.com/TuSimple/octopus-gps/raw/master/analysis/img/prompt1.png) 

5. Input the topic name, then press Enter (Use GPS as example). 

![prompt2](https://github.com/TuSimple/octopus-gps/raw/master/analysis/img/prompt2.png)

5. Wait for the analysis to be completed!

## Parameter Configuration

* SAMPLE_NOV: This parameter is in the “util/vsimple/plot/BagToMap.py” file. This parameter is used to decide the sample rate applied on the Novatel data when painting on Google Map. For VSIMPLE part ONLY! Default value is 100.

* SAMPLE_GPS: This parameter is in the “util/vsimple/plot/BagToMap.py” file. This parameter is used to decide the sample rate applied on the test GPS data when painting on Google Map. For VSIMPLE part ONLY! Default value is 100.

* NO_RESULT_THRE: This parameter is in the “util/vsimple/plot/BagToMap.py” file. This parameter is used to reject the case which the output is in the "No result" state. Default value is 1000(m).

* SAMPLE_NUM: This parameter is in the “util/vsimple/plot/BagToMap.py” file. This parameter is used to decide the sample rate applied on the test GPS data. Default value is 1(i.e. use every data point).

* MATCH_THRE: This parameter is in the “util/vsimple/plot/BagToMap.py” file. This parameter is used to decide the time match threshold between test GPS data and Novatel data. Default value is 10000000(i.e. 0.01s).
