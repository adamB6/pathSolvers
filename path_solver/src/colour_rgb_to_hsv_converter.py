#!/usr/bin/env python
import numpy as np
import cv2

# get input of R,G,B values
R = input("RGB, Red = ")
G = input("RGB, Green = ")
B = input("RGB, Blue = ")

# store in numpy array
yellow = np.uint8([[[B,G,R]]])

# convert to HSV
hsv_yellow = cv2.cvtColor(yellow, cv2.COLOR_BGR2HSV)
print(hsv_yellow)
