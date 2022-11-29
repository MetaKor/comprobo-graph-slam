import numpy as np
import cv2 as cv
from sklearn.cluster import KMeans
from collections import Counter
from objectclass import objectdetect

#TODO - Load all objects from folder
# Initialize primary color of each object
# Search for each object in video stream and determine if object is in frame
# If object is in frame, find the centroid and then


class objecttrack():
    def __init__(self):
        # create a placeholder for each object instance, file name, and diameter
        self.obj1 = None
        self.dia1 = None
        self.name1 = None
        self.obj2 = None
        self.dia2 = None
        self.name2 = None
        self.obj3 = None
        self.dia3 = None
        self.name3 = None
        self.obj4 = None
        self.dia4 = None
        self.name4 = None
        self.obj5 = None
        self.dia5 = None
        self.name5 = None

    def load_images(self):
        pass
