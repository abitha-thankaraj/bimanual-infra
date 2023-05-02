import numpy as np
import numpy as np
from scipy.spatial.transform import Rotation
from bimanual.servers.state_server import get_homogenous_inv

if __name__ == "__main__":
    # generate a random rotation matrix
    home_rotation = Rotation.random().as_matrix()
    translation_home = np.array([1., 2., 3.])
    translation_dest = np.array([4., 5., 6.])
    
    home_homogenous_coords = np.block([[home_rotation, translation_home[:, np.newaxis]],[np.array([0,0,0,1])]])

    dest_homogenous_coords  = np.block([[home_rotation, translation_dest[:, np.newaxis]],[np.array([0,0,0,1])]])


    relative_homogenous_coords = dest_homogenous_coords @ get_homogenous_inv(home_homogenous_coords)
    from IPython import embed; embed()
    # generate a random rotation matrix
    robot_rotation = Rotation.random().as_matrix()

    robot_translation_home = np.array([4., 5., 6.])
    robot_translation_dest = np.array([7., 8., 9.])
    
    
    robot_home_affine = np.block([[robot_rotation, robot_translation_home[:, np.newaxis]],[np.array([0,0,0,1])]])

    robot_dest_affine = np.block([[robot_rotation, robot_translation_dest[:, np.newaxis]],[np.array([0,0,0,1])]])

    # dest_affine = np.block([[rand_rotation, translation_dest[:, np.newaxis]],[np.array([0,0,0,1])]])
    
    
    
    from IPython import embed; embed()