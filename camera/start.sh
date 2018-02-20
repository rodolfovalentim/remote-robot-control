#!/usr/bin/env bash

export LD_LIBRARY_PATH=/usr/local/lib/mjpg-streamer/
./mjpg-streamer/mjpg-streamer-experimental/_build/mjpg_streamer -o "output_http.so -w ./www" -i "input_raspicam.so -fps 15 -cfx 128:128"
