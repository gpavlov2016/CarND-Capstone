import numpy as np
import glob
import cv2

from skimage.io import imread
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split, cross_val_score
from sklearn.feature_selection import SelectFromModel
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.pipeline import Pipeline
from scipy.ndimage.measurements import label
from sklearn.externals import joblib
import pickle

# Setup global variables
RED_LIGHT_LOCATION = "../data/training_data/red_lights/*.png"
GREEN_LIGHT_LOCATION = "../data/training_data/green_lights/*.png"
YELLOW_LIGHT_LOCATION = "../data/training_data/yellow_lights/*.png"
NO_LIGHT_LOCATION = "../data/training_data/no_lights/*.png"


CLASS_UNKNOWN=4
CLASS_GREEN=2
CLASS_YELLOW=1
CLASS_RED=0

CLASSES = [CLASS_RED, CLASS_GREEN, CLASS_YELLOW, CLASS_UNKNOWN]
IMAGES = [glob.glob(RED_LIGHT_LOCATION), glob.glob(GREEN_LIGHT_LOCATION), glob.glob(YELLOW_LIGHT_LOCATION), glob.glob(NO_LIGHT_LOCATION)]

IMG_SHAPE = (30, 60)

class TrafficLightClassifier:
    def __init__(self):
        self.clf = None

    def set_model(self, clf):
        self.clf = clf

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

    def load_images_and_extract_features(self,
                                         cspace="RGB"):
        """
        Loads images and extracts the features, returning X and Y arrays.
        :param vehicle_images: Glob to find the vehicle images.
        :param non_vehicle_images: Glob to find the non-vehicle images.
        :param cspace: If not using RGB, supply a different color space.
        :return:
        """

        print("Loading images and extracting features...")

        Y = []
        X = []

        for idx, classification in enumerate(CLASSES):
            for image_uri in IMAGES[idx]:
                image = cv2.imread(image_uri)

                # apply color conversion if other than 'RGB'
                if cspace != 'RGB':
                    if cspace == 'HSV':
                        feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
                    elif cspace == 'LUV':
                        feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2LUV)
                    elif cspace == 'HLS':
                        feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
                    elif cspace == 'YUV':
                        feature_image = cv2.cvtColor(image, cv2.COLOR_RGB2YUV)
                else:
                    feature_image = np.copy(image)

                h = self.extract_features_from_image(np.resize(image, (30, 60, 3)))
                Y.append(classification)
                X.append(h.astype(np.float64))

        Y = np.array(Y)
        X = np.array(X)

        print("Calculated features on {0} images".format(len(X)))

        return X, Y

    def create_classifier(self):
        return Pipeline([
            ('scaling', StandardScaler(with_mean=0, with_std=1)),
            ('feature_selection', SelectFromModel(ExtraTreesClassifier())),
            ('classification', SVC(kernel="rbf", verbose=1, probability=True))
        ])


def train():
    tl_classifier = TrafficLightClassifier()

    # Gather the data and split into train and test data.
    X, Y = tl_classifier.load_images_and_extract_features()
    x_train, x_test, y_train, y_test = train_test_split(X, Y, test_size=0.3)



    # Create train the classifier
    clf = tl_classifier.create_classifier()

    print("Fitting SVM")
    clf.fit(x_train, y_train)
    acc = accuracy_score(clf.predict(x_test), y_test)
    print("SVM accuracy: {0}".format(acc))

    # Save the model and data
    joblib.dump(clf, 'svm.p')
    pickle.dump((x_train, x_test, y_train, y_test), open("xxyy.p", "wb"))

def main():
    pass
    # Load the model
    # clf = joblib.load("svm.p")
    # vehicle_detector = VehicleDetector()
    # vehicle_detector.set_model(clf)
    #
    # vehicle_detector.process_video("project_video.mp4", "project_video_out.mp4")


if __name__ == "__main__":
    train()
    # main()