from sklearn.cluster import KMeans
from scipy import stats as st


# class for managing visual objects to track

class objectdetect():


    def __init__(self, image, diameter, name):
        # feed in the training image of the object to track, already read
        # by opencv when fed into objectdetect
        self.image = image
        # create a placeholder for the primary color of the object
        # note color is a list of floats in BGR format
        self.primary_color = None
        # create a placeholder for the corners of the object
        self.diameter = diameter
        # create a placeholder for the name of the object
        self.name = name

    def find_color(self):
        # convert the image to a data matrix pixels * RGB
        image_matrix = self.image.reshape(self.image.shape[0]*self.image.shape[1], 3)

        # find the clusters
        clusters = KMeans(n_clusters=4)
        # count which cluster has the most points
        labels = clusters.fit_predict(image_matrix)

        # count which cluster is the most dominant
        label_count = st.mode(labels)

        # find the most popular color (centroid)
        dominant_color = clusters.cluster_centers_[label_count.mode[0]]

        self.primary_color = dominant_color
