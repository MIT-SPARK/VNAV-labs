#!/usr/bin/env python

import pandas as pd
import sys
import getopt

from tesse_ros_bridge.utils import get_enu_T_brh, get_translation_part, get_quaternion


def main(argv):

    inputfile = ''
    outputfile = ''

    # parse arguments
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print 'usage: convert_to_.py -i <inputfile> -o <outputfile>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'usage: python convert_to_.py -i <inputfile> -o <outputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-o", "--ofile"):
            outputfile = arg

    in_df = pd.read_csv(inputfile,
                        index_col=0,
                        header=None,
                        names=['name', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw']
    )
    out_df = pd.DataFrame(columns=['name', 'px', 'py', 'pz', 'qx', 'qy', 'qz', 'qw'])

    for index, row in in_df.iterrows():
        pos = [row['px'], row['py'], row['pz']]
        quat = [row['qx'], row['qy'], row['qz'], row['qw']]

        data = {'position': pos, 'quaternion': quat}
        enu_T_brh = get_enu_T_brh(data)
        enu_t_brh = get_translation_part(enu_T_brh)
        enu_q_brh = get_quaternion(enu_T_brh)

        new_row = {"name": index,
                   "px": enu_t_brh[0],
                   "py": enu_t_brh[1],
                   "pz": enu_t_brh[2],
                   "qx": enu_q_brh[0],
                   "qy": enu_q_brh[1],
                   "qz": enu_q_brh[2],
                   "qw": enu_q_brh[3],
        }
        out_df = out_df.append(new_row, ignore_index=True)

    out_df.set_index("name", drop=True, inplace=True)
    out_df.to_csv(outputfile, index=True, header=False)


if __name__ == "__main__":
    main(sys.argv[1:])
