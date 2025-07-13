#!/usr/bin/env python3


"""
evaluator.py
==================

SLAM evaluator.

Author
----------
Bo Miquel Nordfeldt-Fiol

Date
----------
2025
"""


import os
import sys
import argparse
import numpy as np
import tools.file_helpers as f_help


def evaluate(gt_data_path, estimates_data_path, estimates_labels, output_file):
    r"""
    Compare each estimate with the ground truth.

    Parameters
    ----------
    gt_data_path : str
        Path to the file containing ground truth data.
    estimates_data_path : list[str]
        Paths to the files containing the different estimates.
    estimates_labels : list[str]
        The labels of each of the estimates.
    output_file : str
        The file where store the results.

    Returns
    -------
    None
    """

    # Ground truth sanity check.
    if not f_help.isFull(gt_data_path):
        print("GT is empty, exiting!")
        return

    # Checks gt file type.
    cols, symbol = f_help.checkFileType(gt_data_path)

    # Reads gt data.
    try:
        gt_data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = cols)
    except Exception as e:
        print(f"The following exception was detected while trying to load the GT (size {symbol} 12): {e} ")
        if f_help.removeShutdownInTxt(gt_data_path):
            gt_data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = cols)
        else:
            print(f"The problem cannot be fixed automatically.")
            return
        
    last_pose_gt = gt_data[-1]

    for i in range(len(estimates_data_path)):
        
        # Sanity check.
        if not f_help.isFull(gt_data_path):
            print(f"{estimates_labels[i]} is empty, exiting!")
            return
        
        # Checks file type.
        cols, symbol = f_help.checkFileType(estimates_data_path[i])

        # Reads data.
        try: 
            data = np.loadtxt(estimates_data_path[i], delimiter = ",", skiprows = 1, usecols = cols)
        except Exception as e:
            print(f"The following exception was detected while trying to load the {estimates_labels[i]} (size {symbol} 12): {e} ")
            if f_help.removeShutdownInTxt(gt_data_path):
                data = np.loadtxt(estimates_data_path[i], delimiter = ",", skiprows = 1, usecols = cols)
            else:
                print(f"The problem cannot be fixed automatically.")
                return

        # Computes error.
        last_pose = data[-1]
        last_error_pose = np.sqrt((last_pose_gt[1] - last_pose[1])**2 + (last_pose_gt[2] - last_pose[2])**2)

        print(f"{estimates_labels[i]}: {round(last_error_pose, 2)}m")
        
        with open(output_file, "a") as file:
            file.write(f"{estimates_labels[i]} last pose error: {round(last_error_pose, 2)}m\n")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = "SLAM evaluator.",
                                     formatter_class = argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-gt", "--gt", type = str, 
                        help = "Path to the file containing the ground truth data (required).")
    parser.add_argument("-e", "--e", type = str, nargs = "+",
                        help = "Paths to the estimate files (e.g: ../output/vo_0.txt ../output/keyframes_poses_0.txt) (required).")
    parser.add_argument("-l", "--l", type = str, nargs = "+",
                        help = "Labels for the corresponding estimates (e.g: VO V-SLAM) (required).")
    parser.add_argument("-o", "--o", type = str,
                        default = os.path.join(os.getcwd(), "results.txt"),
                        help = "Output file to store the results.")
    args = parser.parse_args()

    # Sanity checks.
    if not f_help.checkPathExistence(args.gt, "GT"):
        sys.exit()
    for i in range(len(args.e)):
        if not f_help.checkPathExistence(args.e[i], args.l[i]):
            sys.exit()

    # Evaluates.
    evaluate(args.gt, args.e, args.l, args.o)