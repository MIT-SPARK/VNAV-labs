#####################################################################
import numpy as np

# Module-scoped transformation matrices:
enu_T_unity = np.array([[1,0,0,0],
                        [0,0,1,0],
                        [0,1,0,0],
                        [0,0,0,1]])
unity_T_enu = np.transpose(enu_T_unity)

brh_T_blh = np.array([[1,0,0,0],
                      [0,-1,0,0],
                      [0,0,1,0],
                      [0,0,0,1]])
blh_T_brh = np.transpose(brh_T_blh)

rotx_90 = np.array([[1,0, 0,0],
                    [0,0,-1,0],
                    [0,1, 0,0],
                    [0,0, 0,1]])
rotz_90 = np.array([[0,-1,0,0],
                    [1, 0,0,0],
                    [0, 0,1,0],
                    [0, 0,0,1]])
brh_T_bros = np.matmul(rotx_90, rotz_90)
bros_T_brh = np.transpose(brh_T_bros)

gravity_enu = [0.0, 0.0, -9.81] # in 'world' frame
#####################################################################