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
from plyfile import PlyData
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Global variables
ax_gt = None
ax_re = None
ax_vo = None
ax_ekf = None
plot_dim = 3
ax_edges = []
ax_vertices = None
edges_shown = True
ax_pipes_map = None
ax_pipes_odom = None
legend_edited = True
edges_lock_file = ""
vertices_lock_file = ""
loop_closing_edges_data_path = ""


def str2bool(v):
  r"""
  Convert a string or boolean input to a proper boolean value.
  
  This is useful for argparse to interpret command-line flags like:
    --flag True
    --flag false
    --flag 1
    --flag no
  
  Accepted truthy values: 'yes', 'true', 'y', 't', '1'
  Accepted falsy values: 'no', 'false', 'n', 'f', '0'

  Parameters
  ----------
  v : str or bool
    Input value to convert.

  Returns
  -------
  out : bool
    Converted boolean value.
  """
  if isinstance(v, bool):
    return v
  if v.lower() in ('yes', 'true', 'y', 't', '1'):
    return True
  elif v.lower() in ('no', 'false', 'n', 'f', '0'):
    return False
  else:
    raise argparse.ArgumentTypeError("Boolean value expected.")


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
    True if the file length is greater than 0. False otherwise.
  """
  with open(path_to_file, "r") as file:
    lines = file.readlines()
  return len(lines) > 0


def removeShutdownInTxt(path_to_file):
  r"""
  Deletes the last txt line if it contains the string: shutdown

  Parameters
  ----------
  path_to_file : str
    Path to the file to be analyzed.

  Returns
  -------
  out : Bool
    True if the last line has been removed.
  """
  with open(path_to_file, "r") as file:
    lines = file.readlines()
  if not "shutdown" in lines[-1]:
    return False
  lines.pop()
  with open(path_to_file, "w") as file:
    file.writelines(lines)
  return True


def colorGradient(value, min_val, max_val, color_start, color_end):
  r"""
  Interpolates between two RGB hex colors based on the value within a range.

  Parameters
  ----------
    value : float
      The current confidence/inlier value.
    min_val : float
      Minimum confidence value.
    max_val : float
      Maximum confidence value.
    color_start : tuple
      RGB tuple for low values, e.g., (255, 204, 204).
    color_end : tuple
      RGB tuple for high values, e.g., (255, 0, 0).
      
  Returns
  -------
  out : str
    Hex color string like '#ffcccc'
  """
  t = (value - min_val) / (max_val - min_val) if max_val != min_val else 1
  r = int(color_start[0] * (1 - t) + color_end[0] * t)
  g = int(color_start[1] * (1 - t) + color_end[1] * t)
  b = int(color_start[2] * (1 - t) + color_end[2] * t)
  return f'#{r:02x}{g:02x}{b:02x}'


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


def readPLYVertices(path_to_file):
  r"""
  Reads .ply files and returns their vertices.

  Parameters
  ----------
  path_to_file : str
    Path to the .ply file to be read.

  Returns
  ----------
  x : numpy.ndarray[numpy.float32]
    x-coordinates of the vertices.
  y : numpy.ndarray[numpy.float32]
    y-coordinates of the vertices.
  """
  with open(path_to_file, "rb") as file:
    ply_data = PlyData.read(file)
  vertices = ply_data['vertex'].data
  return vertices['x'], vertices['y']


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
    min_conf, max_conf = np.min(confidences), np.max(confidences)

    # Plot intra loop closures edges.
    for i, edge in enumerate(data):
      color = colorGradient(edge[2], min_conf, max_conf, (255, 204, 204), (255, 0, 0))
      vect = np.vstack((edge[3:6], edge[6:9]))
      if plot_dim == 3:
          ax_edge = ax.plot(vect[:, 0], vect[:, 1], vect[:, 2], color = color, linestyle = "--",
                            label = "Intra-loop closing edges" if i == 0 else "", zorder = 1)
      else:
          ax_edge = ax.plot(vect[:, 0], vect[:, 1], color = color, linestyle = "--",
                            label = "Intra-loop closing edges" if i == 0 else "", zorder = 1)
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


def plotOnline(gt_data_path, re_data_path, vo_data_path, slam_data_path, pipes_folder_path):
  r"""
  Plots trajectory data from ground truth, robot, visual odometry, and SLAM sources. Also plots pipes positions.

  Parameters
  ----------
  gt_data_path : str
    Path to the file containing ground truth trajectory data.
  re_data_path : str
    Path to the file containing the robot trajectory estimation data.
  vo_data_path : str
    Path to the file containing visual odometry trajectory data.
  slam_data_path : str
    Path to the file containing SLAM trajectory data.
  pipes_folder_path : str
    Path to the folder where the pipeline locations are stored.

  Returns
  ----------
  None
  """
  global vertices_lock_file, legend_edited, ax_gt, ax_re, ax_vo, ax_vertices, ax_pipes_map, ax_pipes_odom, edges_shown, plot_dim

  # Remove the main axes
  removeAxes(ax_gt)
  removeAxes(ax_re)
  removeAxes(ax_vo)
  removeAxes(ax_vertices)
  removeAxes(ax_pipes_map)
  removeAxes(ax_pipes_odom)

  # Ground truth.
  if (gt_data_path is not None and os.path.isfile(gt_data_path) and isFull(gt_data_path)):

    # Checks gt file type.
    with open(gt_data_path, "r") as file:
      lines = file.readlines()
    size = lines[1].split(",")
    if (len(size) > 12):
      cols = (0, 5, 6, 7, 8, 9, 10, 11)
      symbol = ">"
    else:
      cols = (0, 2, 3, 4, 5, 6, 7, 8)
      symbol = "<="

    # Reads data
    try:
      data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = cols)
    except Exception as e:
      print(f"The following exception was detected while trying to load the GT (size {symbol} 12): {e} ")
      if removeShutdownInTxt(gt_data_path):
        data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = cols)
      else:
        return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])
  
    # Plot.
    if (plot_dim == 3):
      ax_gt = ax.plot(data[:, 1], data[:, 2], data[:, 3], color = "black", linestyle = "-", label = "Ground Truth")
    else:
      ax_gt = ax.plot(data[:, 1], data[:, 2], color = "black", linestyle = "-", label = "Ground Truth")

  # Robot estimation.
  if (re_data_path is not None and os.path.isfile(re_data_path) and isFull(re_data_path)):

    # Checks re file type.
    with open(re_data_path, "r") as file:
      lines = file.readlines()
    size = lines[1].split(",")
    if (len(size) > 12):
      cols = (0, 5, 6, 7, 8, 9, 10, 11)
      symbol = ">"
    else:
      cols = (0, 2, 3, 4, 5, 6, 7, 8)
      symbol = "<="

    # Reads data
    try:
      data = np.loadtxt(re_data_path, delimiter = ",", skiprows = 1, usecols = cols)
    except Exception as e:
      print(f"The following exception was detected while trying to load the robot's location estimation (size {symbol} 12): {e} ")
      if removeShutdownInTxt(re_data_path):
        data = np.loadtxt(re_data_path, delimiter = ",", skiprows = 1, usecols = cols)
      else:
        return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])
  
    # Plot.
    if (plot_dim == 3):
      ax_re = ax.plot(data[:, 1], data[:, 2], data[:, 3], color = "brown", linestyle = "-", label = "Robot's location estimation")
    else:
      ax_re = ax.plot(data[:, 1], data[:, 2], color = "brown", linestyle = "-", label = "Robot's location estimation")

  # Load visual odometry data (saved with rostopic echo -p /namespace/topic > file.txt).
  if (vo_data_path is not None and os.path.isfile(vo_data_path) and isFull(vo_data_path)):

    # Reads data
    try:
      data = np.loadtxt(vo_data_path, delimiter = ",", skiprows = 1, usecols = (5, 6, 7, 8, 9, 10, 11))
    except Exception as e:
      print(f"The following exception was detected while trying to load the vo: {e} ")
      if removeShutdownInTxt(vo_data_path):
        data = np.loadtxt(vo_data_path, delimiter = ",", skiprows = 1, usecols = (5, 6, 7, 8, 9, 10, 11))
      else:
        return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_vo = ax.plot(data[:, 0], data[:, 1], data[:, 2], color = "darkturquoise", linestyle = "-", label = "VO")
    else:
      ax_vo = ax.plot(data[:, 0], data[:, 1], color = "darkturquoise", linestyle = "-", label = "VO")

  # Load SLAM vertices (file saved with SLAM node)
  if (os.path.isfile(slam_data_path) and isFull(slam_data_path)):

    # Check if file is blocked.
    while (os.path.exists(vertices_lock_file)):
      time.sleep(0.5)

    # Read the data
    with open(slam_data_path, "r") as file:
      lines = file.readlines()
    size = lines[1].split(",")
    if len(size) != 9:
      print(f"Wrong number of columns")
      return
    try:
      data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (2, 3, 4, 5, 6, 7, 8))
    except Exception as e:
      print(f"The following exception was detected while trying to load the SLAM algorithm vertices (size 9): {e} ")
      if removeShutdownInTxt(slam_data_path):
        data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (2, 3, 4, 5, 6, 7, 8))
      else:
        return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_vertices = ax.plot(data[:, 0], data[:, 1], data[:, 2], color = "green", linestyle = "-", label = "V-SLAM", marker = "o", zorder = 1)
    else:
      ax_vertices = ax.plot(data[:, 0], data[:, 1], color = "green", linestyle = "-", label = "V-SLAM", marker = "o", zorder = 1)

  # Show the edges
  if (edges_shown == True):
    drawEdges()
  else:
    removeEdges()

  # Draw pipes.
  if pipes_folder_path is not None and os.path.isdir(pipes_folder_path):
    
    # Load pipes.
    pipes_map_x = []
    pipes_map_y = []
    pipes_odom_x = []
    pipes_odom_y = []
    pipes_map_names = []
    for file_name in os.listdir(pipes_folder_path):
      if file_name.endswith("odom.ply"):
        x, y = readPLYVertices(os.path.join(pipes_folder_path, file_name))
        pipes_odom_x.extend(x)
        pipes_odom_y.extend(y)
      if file_name.endswith("map.ply") and not "info" in file_name:
        pipes_map_names.append(file_name)
    if (len(pipes_map_names) > 0):
      pipes_map_x, pipes_map_y = readPLYVertices(os.path.join(pipes_folder_path, pipes_map_names[0]))

    # Plot pipes.
    ax_pipes_odom = ax.plot(pipes_odom_x, pipes_odom_y, color = "gold", linestyle = 'None', label = "Pipes first detection", marker = "o", markersize = 2)
    ax_pipes_map = ax.plot(pipes_map_x, pipes_map_y, color = "orange", linestyle = 'None', label = "Pipes corrected", marker = "o", markersize = 2)

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
  parser.add_argument("-re", "--re", type = str,
                      default = None,
                      help = "Path to the file containing the robot's location estimate (optional)." )
  parser.add_argument("-vo", "--vo", type = str, 
                      default = None,
                      help = "Path to the file containing the visual odometry data (optional).")
  parser.add_argument("-v", "--v", type = str, 
                      default = None,
                      help = "Path to the file containing the SLAM vertices data (required).")
  parser.add_argument("-e", "--e", type = str,
                      default = None,
                      help = "Path to the file containing the SLAM loop closing edges data (optional).")
  parser.add_argument("-p", "--p", type = str,
                      default = None,
                      help = "Path to the folder where the pipeline locations are stored (optional).")
  parser.add_argument("-dim", "--dim", type = int,
                      default = 2,
                      help = "Defines the plot dimensions: 2 for xy and 3 for xyz.")
  parser.add_argument("-ned", "--ned", type = str2bool, 
                      default = True,
                      help = "Select NED (True) or ENU (False) axis.")
  args = parser.parse_args()

  # Sanity check
  if (args.v):
    if (not os.path.isfile(args.v)):
      print("Vertices is not a file.")
      sys.exit()
  else:
    print("Vertices is None.")
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
    if args.ned:
      ax.view_init(elev = -90, azim = 0)
      ax.invert_zaxis()
  else:
    ax = fig.add_subplot(111)
    ax.grid()
    if args.ned:
      ax.invert_yaxis()

  ax.set_title("Online viewer")
  ax.set_xlabel("x (m)")
  ax.set_ylabel("y (m)")

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onClick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  plotOnline(args.gt, args.re, args.vo, args.v, args.p)
  timer.add_callback(plotOnline, args.gt, args.re, args.vo, args.v, args.p)
  timer.start()
  plt.show()