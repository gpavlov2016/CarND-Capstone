from styx_msgs.msg import TrafficLight
import numpy as np
from sklearn.externals import joblib

CLASS_UNKNOWN=4
CLASS_GREEN=2
CLASS_YELLOW=1
CLASS_RED=0

class TLClassifier(object):
    def __init__(self):
        # Load the model
        self.clf = joblib.load("/capstone/model/svm.p")

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        x = self.extract_features_from_image(image)
        prediction = self.clf.predict([x])[0]


        if (prediction == CLASS_GREEN):
            return TrafficLight.GREEN
        elif (prediction == CLASS_RED):
            return TrafficLight.RED
        elif (prediction == CLASS_YELLOW):
            return TrafficLight.YELLOW
        else:
            TrafficLight.UNKNOWN


    # Define a function to compute color histogram features (from udacity lectures).
    def color_hist(self, img, nbins=32, bins_range=(0, 256)):
        """
        Computes histogram features on each channel in the image, returning a single flat array of features.
        """
        # Compute the histogram of the color channels separately
        channel1_hist = np.histogram(img[:, :, 0], bins=nbins, range=bins_range)
        channel2_hist = np.histogram(img[:, :, 1], bins=nbins, range=bins_range)
        channel3_hist = np.histogram(img[:, :, 2], bins=nbins, range=bins_range)
        # Concatenate the histograms into a single feature vector
        return np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))

    def extract_features_from_image(self, image):
        """
        This method runs HOG only, because bin spatial and color histograms didn't add enough accuracy to justify the
        number of features.
        """
        return self.color_hist(image)

