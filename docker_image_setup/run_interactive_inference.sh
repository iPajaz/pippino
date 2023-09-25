#!/bin/bash

docker stop pippino_inference
docker rm pippino_inference

cd ~/ai/jetson-inference

./docker/run.sh -c dustynv/ros:humble-pytorch-l4t-r32.7.1

