from collections import OrderedDict
import numpy as np
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment
from multi_object_tracker.motrackers.tracker import Tracker
from multi_object_tracker.motrackers.track import KFTrackCentroid
from multi_object_tracker.motrackers.utils.misc import get_centroid


