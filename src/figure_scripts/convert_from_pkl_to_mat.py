import pickle
import numpy, scipy.io
import sys

# reload(sys)
# sys.setdefaultencoding('utf-8')

filename = '/Users/somil/Desktop/Experimental_results/successful/without_safety/rgb_resnet50_nn_waypoint_simulator/trajectories/traj_0.pkl'
data=pickle.load(open(filename, "rb"), encoding='latin1')
pos_heading_k3 = numpy.concatenate([data['vehicle_trajectory']['position_nk2'][0], data['vehicle_trajectory']['heading_nk1'][0]], axis=1)
scipy.io.savemat('pos_heading.mat', mdict={'data': pos_heading_k3})
