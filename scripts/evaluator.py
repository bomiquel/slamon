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
import pandas as pd
import tools.file_helpers as f_help
from scipy.interpolate import interp1d


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

    # Checks gt file type.
    cols, symbol, time_in_seconds = f_help.checkFileType(gt_data_path)

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
        
    if not time_in_seconds:
        data[:, 0] = data[:, 0] / 1000000000
        
    last_pose_gt = gt_data[-1]

    # Interpolate functions.
    gt_interp_funcs = {
        'x': interp1d(gt_data[:, 0], gt_data[:, 1], kind = 'linear', bounds_error = False),
        'y': interp1d(gt_data[:, 0], gt_data[:, 2], kind = 'linear', bounds_error = False),
    }

    for i, estimate_data_path in enumerate(estimates_data_path):
        
        # Checks file type.
        cols, symbol, time_in_seconds = f_help.checkFileType(estimate_data_path)

        # Reads data.
        try: 
            data = np.loadtxt(estimate_data_path, delimiter = ",", skiprows = 1, usecols = cols)
        except Exception as e:
            print(f"The following exception was detected while trying to load the {estimate_data_path} (size {symbol} 12): {e} ")
            if f_help.removeShutdownInTxt(estimate_data_path):
                data = np.loadtxt(estimate_data_path, delimiter = ",", skiprows = 1, usecols = cols)
            else:
                print(f"The problem cannot be fixed automatically.")
                return
        
        if not time_in_seconds:
            data[:, 0] = data[:, 0] / 1000000000

        # Computes LPE.
        last_pose = data[-1]
        lpe = np.sqrt((last_pose_gt[1] - last_pose[1])**2 + (last_pose_gt[2] - last_pose[2])**2)

        # Filter estimates within the time range of the ground truth.
        valid_mask = (
            (data[:, 0] >= gt_data[:, 0].min()) &
            (data[:, 0] <= gt_data[:, 0].max())
        )
        data_valid = data[valid_mask].copy()

        # Interpolate ground truth at estimate timestamps.
        timestamps = data_valid[:, 0]
        gt_interp = np.stack([
            gt_interp_funcs['x'](timestamps),
            gt_interp_funcs['y'](timestamps),
        ], axis=1)

        # Compute translation ATE.
        trans_errors = np.linalg.norm(gt_interp - data_valid[:, 1:3], axis=1)
        ate = np.mean(trans_errors)
        ate_rmse = np.sqrt(np.mean(trans_errors**2))

        print(f"{estimates_labels[i]}: LPE {round(lpe, 2)}m, ATE {round(ate, 2)}m and ATE-RMSE {round(ate_rmse, 2)}m")

        with open(output_file, "a") as file:
            file.write(f"{estimates_labels[i]}: LPE {round(lpe, 2)}m, ATE {round(ate, 2)}m and ATE-RMSE {round(ate_rmse, 2)}m\n")


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description = "SLAM evaluator.",
                                     formatter_class = argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-gt", "--gt", type = str, 
                        help = "Path to the file containing the ground truth data (required).")
    parser.add_argument("-ests", "--ests", type = str, nargs = "+",
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
    if not f_help.isFull(args.gt):
        print("GT is empty, exiting!")
        sys.exit()
    f_help.removeShutdownInTxt(args.gt)
    if not len(args.ests) == len(args.l):
        print("The number of estimates is not the same as the number of labels, exiting!")
        sys.exit()
    for i in range(len(args.ests)):
        if not f_help.checkPathExistence(args.ests[i], args.l[i]):
            sys.exit()
        if not f_help.isFull(args.ests[i]):
            print(f"{args.l[i]} is empty, exiting!")
            sys.exit()
        f_help.removeShutdownInTxt(args.ests[i])

    evaluate(args.gt, args.ests, args.l, args.o)