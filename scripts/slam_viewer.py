#!/usr/bin/env python3


"""
slam_viewer.py
==================

Online SLAM trajectory viewer.

Author
----------
Pep Lluís Negre-Carrasco

Modifier
----------
Bo Miquel Nordfeldt-Fiol

Date
----------
2014-2016

Modification date
----------
2024-2025
"""


import os
import sys
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Global variables
ax_gt = None
ax_vo = None
ax_ekf = None
plot_dim = 3
ax_edges = []
ax_vertices = None
edges_shown = True
legend_edited = True
edges_lock_file = ""
vertices_lock_file = ""
loop_closing_edges_data_path = ""


def isFull(path_to_file):
  r"""
  Check if the file length is > 0.

  Parameters
  ----------
  path_to_file : str
    Path to the file to be analyzed.

  Returns
  -------
  out : bool
    True if the file length is greater than 0, False otherwise.
  """
  with open(path_to_file, "r") as file:
    lines = file.readlines()
  return len(lines) > 0


def removeAxes(ax):
  r"""
  Fully removes and deletes Line2D objects from a Matplotlib plot.

  Parameters
  ----------
  ax : list[matplotlib.lines.Line2D]
    Axis lines to be removed.

  Returns
  -------
  None
  """
  if ax is not None and ax:
    for i, line in enumerate(ax):
      line.remove() 
      ax[i] = None
    ax.clear()


def updateLegend():
  r""" 
  Updates legend.

  Returns
  -------
  None
  """
  global legend_edited
  ax.legend()
  legend_edited = False


def removeEdges():
  r""" 
  Removes the loop's closing edges.

  Returns
  -------
  None
  """
  global ax_edges, edges_shown
  for ax_edge in ax_edges:
    removeAxes(ax_edge)
  ax_edges.clear()
  edges_shown = False


def drawEdges():
  r""" 
  Draws the loop's closing edges.

  Returns
  -------
  None
  """
  global edges_lock_file, loop_closing_edges_data_path, ax_edges, edges_shown, plot_dim

  # First, remove previous edges.
  removeEdges()

  # Load SLAM edges (file saved with SLAM node)
  if (loop_closing_edges_data_path is not None and os.path.isfile(loop_closing_edges_data_path) and isFull(loop_closing_edges_data_path)):

    # Check if file is blocked
    while (os.path.exists(edges_lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = np.loadtxt(loop_closing_edges_data_path, delimiter = ",", skiprows = 1, usecols = (0, 1, 2, 3, 4, 5, 10, 11, 12))
    except Exception as e:
      print(f"The following exception was detected while trying to load the SLAM algorithm edges: {e} ")
      return

    # Sanity check.
    if (data.size == 0):
      edges_shown = True
      return

    # Sanity check 2.
    if (len(data.shape) == 1):
      data = np.array([data])
    
    # Confidence column.
    confidences = data[:,2]
    max_confidence = max(confidences)
    min_confidence = min(confidences)

    # Check for valid color mapping.
    valid_color = (max_confidence - min_confidence != 0)

    # Pink base value for green and blue.
    pink_gb_start = 204
    
    # Compute mapping for green and blue if valid.
    if valid_color:
      m_gb = -pink_gb_start / (max_confidence - min_confidence)
      n_gb = pink_gb_start - m_gb * min_confidence

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    ax_edges = []
    for i in range(len(data)):
      if (valid_color):
        gb_value = int(m_gb * data[i, 2] + n_gb)
        gb_value = max(0, min(pink_gb_start, gb_value))
        gb_hex = f'{gb_value:02x}'
        color = f'#ff{gb_hex}{gb_hex}'
      else:
        color = "#ff0000"
      vect = []
      vect.append([data[i, 3], data[i, 4], data[i, 5]])
      vect.append([data[i, 6], data[i, 7], data[i, 8]])
      vect =  np.array(vect)
      if (plot_dim == 3):
        if i == 0:
          ax_edge = ax.plot(vect[:, 0], vect[:, 1], vect[:, 2], color, label = "Loop closing edges", linestyle = "-", zorder = 1)
        else:
          ax_edge = ax.plot(vect[:, 0], vect[:, 1], vect[:, 2], color, linestyle = "-", zorder = 1)
      else:
        if i == 0:
          ax_edge = ax.plot(vect[:, 0], vect[:, 1], color, label = "Loop closing edges", linestyle = "-", zorder = 1)
        else:
          ax_edge = ax.plot(vect[:, 0], vect[:, 1], color, linestyle = "-", zorder = 1)
      ax_edges.append(ax_edge)

  edges_shown = True

  return


def onClick(event):
  r""" 
  Handle the click event

  Parameters
  ----------
  event : matplotlib.backend_bases.Event
    The mouse click event object.

  Returns
  ----------
  None
  """
  global edges_shown

  if (event.button == 3):
    if (edges_shown):
      removeEdges()
    else:
      drawEdges()
    updateLegend()
    plt.draw()


def plotOnline(gt_data_path, vo_data_path, slam_data_path):
  r"""
  Plots trajectory data from ground truth, visual odometry, and SLAM sources.

  Parameters
  ----------
  gt_data_path : str
    Path to the file containing ground truth trajectory data.
  vo_data_path : str
    Path to the file containing visual odometry trajectory data.
  slam_data_path : str
    Path to the file containing SLAM trajectory data.

  Returns
  ----------
  None
  """
  global vertices_lock_file, legend_edited, ax_gt, ax_vo, ax_vertices, edges_shown, plot_dim

  # Remove the main axes
  removeAxes(ax_gt)
  removeAxes(ax_vo)
  removeAxes(ax_vertices)

  # Ground truth.
  if (gt_data_path is not None and os.path.isfile(gt_data_path) and isFull(gt_data_path)):

    # Check gt file type.
    with open(gt_data_path, "r") as file:
      lines = file.readlines()
    size = lines[1].split(",")

    # Read the data.
    if (len(size) > 12):
      try:
        data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = (0, 5, 6, 7, 8, 9, 10, 11))
      except Exception as e:
        print(f"The following exception was detected while trying to load the GT (size > 12): {e} ")
        with open(gt_data_path, "r") as file:
          lines = file.readlines()
        if "shutdown" in lines[-1]:
          lines.pop()
          with open(gt_data_path, "w") as file:
            file.writelines(lines)
          data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = (0, 5, 6, 7, 8, 9, 10, 11))
        return
    else:
      try:
        data = np.loadtxt(gt_data_path, delimiter=',', skiprows = 1, usecols = (0, 1, 2, 3, 4, 5, 6, 7))
      except Exception as e:
        print(f"The following exception was detected while trying to load the GT (size <= 12): {e} ")
        with open(gt_data_path, "r") as file:
          lines = file.readlines()
        if "shutdown" in lines[-1]:
          lines.pop()
          with open(gt_data_path, "w") as file:
            file.writelines(lines)
          data = np.loadtxt(gt_data_path, delimiter=',', skiprows = 1, usecols = (0, 1, 2, 3, 4, 5, 6, 7))
        return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])
  
    # Plot.
    if (plot_dim == 3):
      ax_gt = ax.plot(data[:, 1], data[:, 2], data[:, 3], "orange", label = "Ground Truth")
    else:
      ax_gt = ax.plot(data[:, 1], data[:, 2], "orange", label = "Ground Truth")

  # Load visual odometry data (saved with rostopic echo -p /namespace/topic > file.txt).
  if (os.path.isfile(vo_data_path) and isFull(vo_data_path)):

    # Read the data.
    try:
      data = np.loadtxt(vo_data_path, delimiter = ",", skiprows = 1, usecols = (5, 6, 7, 8, 9, 10, 11))
    except Exception as e:
      print(f"The following exception was detected while trying to load the vo: {e} ")
      with open(vo_data_path, "r") as file:
        lines = file.readlines()
      if "shutdown" in lines[-1]:
        lines.pop()
        with open(vo_data_path, "w") as file:
          file.writelines(lines)
        data = np.loadtxt(vo_data_path, delimiter = ",", skiprows = 1, usecols = (5, 6, 7, 8, 9, 10, 11))
      return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_vo = ax.plot(data[:, 0], data[:, 1], data[:, 2], "blue", label = "VO")
    else:
      ax_vo = ax.plot(data[:, 0], data[:, 1], "blue", label = "VO")

  # Load SLAM vertices (file saved with SLAM node)
  if (os.path.isfile(slam_data_path) and isFull(slam_data_path)):

    # Check if file is blocked
    while (os.path.exists(vertices_lock_file)):
      time.sleep(0.5)

    # Read the data
    with open(slam_data_path, "r") as file:
      lines = file.readlines()
    size = lines[1].split(",")
    if len(size) == 9:
      try:
        data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (2, 3, 4, 5, 6, 7, 8))
      except Exception as e:
        print(f"The following exception was detected while trying to load the SLAM algorithm vertices (size 9): {e} ")
        with open(slam_data_path, "r") as file:
          lines = file.readlines()
        if "shutdown" in lines[-1]:
          lines.pop()
          with open(slam_data_path, "w") as file:
            file.writelines(lines)
          data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (2, 3, 4, 5, 6, 7, 8))
        return
    elif len(size) == 8:
      try:
        data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (1, 2, 3, 4, 5, 6, 7))
      except Exception as e:
        print(f"The following exception was detected while trying to load the SLAM algorithm vertices (size 8): {e} ")
        with open(slam_data_path, "r") as file:
          lines = file.readlines()
        if "shutdown" in lines[-1]:
          lines.pop()
          with open(slam_data_path, "w") as file:
            file.writelines(lines)
          data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (1, 2, 3, 4, 5, 6, 7))
        return
    else:
      print(f"Wrong number of columns")
      return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_vertices = ax.plot(data[:, 0], data[:, 1], data[:, 2], "green", label = "V-SLAM", marker = "o", zorder = 1)
    else:
      ax_vertices = ax.plot(data[:, 0], data[:, 1], "green", label = "V-SLAM", marker = "o", zorder = 1)

  # Show the edges
  if (edges_shown == True):
    drawEdges()
  else:
    removeEdges()

  # Update the plot
  plt.draw()

  # Show legend only once
  if (ax_vertices is not None and legend_edited is True):
    updateLegend()


if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(description = "Online SLAM trajectory viewer.",
                                   formatter_class = argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("-gt", "--gt", type = str, 
                      default = None,
                      help = "Path to the file containing the ground truth data (optional).")
  parser.add_argument("-vo", "--vo", type = str, 
                      default = None,
                      help = "Path to the file containing the visual odometry data (required).")
  parser.add_argument("-v", "--v", type = str, 
                      default = None,
                      help = "Path to the file containing the SLAM vertices data (required).")
  parser.add_argument("-e", "--e", type = str,
                      default = None,
                      help = "Path to the file containing the SLAM loop closing edges data (optional).")
  parser.add_argument("-dim", "--dim", type = int,
                      default = 2,
                      help = "Defines the plot dimensions: 2 for xy and 3 for xyz.")
  args = parser.parse_args()

  # Some hardcode parameters
  # font = {'family' : 'Sans',
  #         'weight' : 'normal',
  #         'size'   : 30}
  # pylab.rc('font', **font)

  # Sanity check
  if (args.v):
    if (not os.path.isfile(args.v)):
      print("Vertices is not a file.")
      sys.exit()
  else:
    print("Vertices is None.")
    sys.exit()
  if (args.vo):
    if (not os.path.isfile(args.vo)):
      print("VO is not a file.")
      sys.exit()
  else:
    print("VO is None.")
    sys.exit()

  print("GRAPH VIEWER MOUSE INPUTS:")
  print(" - Right button: activates/deactivates the visualization of graph edges.")

  # Save global variables
  plot_dim = args.dim
  if args.e is not None:
    edges_lock_file = args.e + ".lock"
  vertices_lock_file = args.v + ".lock"
  loop_closing_edges_data_path = args.e

  # Init figure
  fig = plt.figure(1)
  if (plot_dim == 3):
    ax = fig.add_subplot(111, projection = '3d')
    ax.set_zlabel("z (m)")
  else:
    ax = fig.add_subplot(111)
    ax.grid()

  ax.set_title("Online viewer")
  ax.set_xlabel("x (m)")
  ax.set_ylabel("y (m)")

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onClick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  plotOnline(args.gt, args.vo, args.v)
  timer.add_callback(plotOnline, args.gt, args.vo, args.v)
  timer.start()
  plt.show()