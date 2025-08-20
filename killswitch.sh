#!/bin/bash

while true; do
    read -p "Press Enter to trigger killswitch..." 
    rosservice call /act/kill
done