import numpy as np
import cv2 as cv
from sklearn.cluster import KMeans
from collections import Counter
from objectclass import objectdetect
import os
from os import listdir
import inspect, os.path
import re

#TODO - Load all objects from folder
# Initialize primary color of each object
# Search for each object in video stream and determine if object is in frame
# If object is in frame, find the centroid and then


class objecttrack():
    def __init__(self, folder_name):
        # create a placeholder to store the tracking objects
        self.objects = None
        # store the name of the folder where the object images are stored
        self.folder_name = folder_name

    def load_objects(self):

        # find the current working directory
        path = os.getcwd() + "/" + self.folder_name

        object_list = []

        for objects in os.listdir(path):

            # the diameter of the image needs to be in the file name ex. greenball_5
            diameter = re.findall("(?<=_).+(?=\.)", objects)[0]

            # attempt to convert the number
            try:
                diameter = float(diameter)
            except NameError:
                print("Object Image Name has no Diameter ending, proper ex. greenball_5")

            # load the image
            img = cv.imread(path + "/" + objects)

            # create an instance of the image
            landmark = objectdetect(img,diameter,objects)

            # find the primary color of the object
            landmark.find_color()

            # create the object class and append it to the list
            object_list.append(landmark)



        # Move the objects to a global placeholder
        self.objects = object_list


test = objecttrack("trackingobjects")

test.load_objects()

print(test.objects[0].name)
print(test.objects[0].primary_color)