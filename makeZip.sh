#!/bin/bash
mkdir tmp
cp cf2.bin tmp/cf2-2018.01.1.bin
cp cf2_nrf-2018.01.bin tmp/cf2_nrf-2018.01.bin
cp manifest.json tmp/manifest.json
cd tmp
zip crazyflie-2018.01.zip cf2-2018.01.1.bin cf2_nrf-2018.01.bin manifest.json
cd ..
cp tmp/crazyflie-2018.01.zip crazyflie-2018.01.zip
rm -r tmp