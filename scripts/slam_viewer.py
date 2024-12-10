#!/usr/bin/env python3

"""
@file
@brief SLAM online viewer.
@author Pep Lluís Negre-Carrasco
@date 2014-2016
@modified Bo Miquel Nordfeldt-Fiol 2024
"""


import os
import sys
import time
import weakref
import argparse
import numpy as np
import matplotlib.pyplot as plt


# Global variables
ax_gt = None
plot_dim = 3
ax_odom = None
ax_edges = []
lock_file = ""
ax_vertices = None
edges_shown = True
legend_edited = True
loop_closing_edges_data_path = ""


def isFull(path_to_file):
  """ 
  Check if the file length is > 0 
  @param path_to_file: path to the file to be analysed.
  @return if file length is > 0.
  """

  f = open(path_to_file)
  lines = f.readlines()
  f.close()
  return len(lines) > 0


def removeAxes(ax_id):
  """ 
  Removes the axes lines.
  @param ax_id: the axes.  
  """

  if (ax_id is not None and ax_id):
    l = ax_id.pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l


def updateLegend():
  """
  Updates legend.
  """
  global legend_edited
  ax.legend()
  legend_edited = False


def removeEdges():
  """ 
  Removes the loop's closing edges.
  """
  global ax_edges, edges_shown

  for i in range(len(ax_edges)):
    l = ax_edges[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l
  ax_edges = []
  edges_shown = False


def drawEdges():
  """ 
  Draws the loop's closing edges.
  """
  global lock_file, loop_closing_edges_data_path, ax_edges, edges_shown, plot_dim

  # First, remove previous edges.
  removeEdges()

  # Load SLAM edges (file saved with SLAM node)
  if (os.path.isfile(loop_closing_edges_data_path) and isFull(loop_closing_edges_data_path)):

    # Check if file is blocked
    while (os.path.exists(lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = np.loadtxt(loop_closing_edges_data_path, delimiter = ",", skiprows = 1, usecols = (0, 1, 2, 3, 4, 9, 10, 11))
    except Exception as e:
      print(f"The following exception was detected while trying to load the SLAM algorithm edges: {e} ")
      return

    # Sanity check
    if (len(data.shape) == 1):
      edges_shown = True
      return

    valid_color = False

    # # Inliers column
    # inliers = data[:,2]
    # max_inliers = max(inliers)
    # min_inliers = min(inliers)

    # valid_color = True
    # if (min_inliers - max_inliers == 0):
    #   valid_color = False

    # if (valid_color):
    #   # red color
    #   m_red = (255) / (min_inliers - max_inliers)
    #   n_red = -m_red * max_inliers

    #   # blue color
    #   m_blue = (255) / (max_inliers - min_inliers)
    #   n_blue = -m_blue * min_inliers

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    ax_edges = []
    for i in range(len(data)):
      # Get the color depending on the inlier value
      if (valid_color):
        # red = hex(int(m_red * data[i,2] + n_red))
        # blue = hex(int(m_blue * data[i,2] + n_blue))
        # red = red[2:]
        # blue = blue[2:]
        # if (len(red) == 1):
        #   red += '0'
        # if (len(blue) == 1):
        #   blue += '0'
        # color = '#' + red + '00' + blue
        color = "green"
      else:
        color = "red"
      vect = []
      vect.append([data[i, 2], data[i, 3], data[i, 4]])
      vect.append([data[i, 5], data[i, 6], data[i, 7]])
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
  """ 
  Handle the click event
  @param event: click event.
  """
  global edges_shown

  if (event.button == 3):
    if (edges_shown):
      removeEdges()
    else:
      drawEdges()
    updateLegend()
    plt.draw()


def plotOnline(gt_data_path, odom_data_path, slam_data_path):
  """ 
  Function to plot the data saved into the files online. 
  @param gt_data_path: Path to the file with the ground truth data.
  @param odom_data_path: Path to the file with the odometry data.
  @param slam_data_path: Path to the file with the slam data.
  """
  global lock_file, legend_edited, ax_gt, ax_odom, ax_vertices, edges_shown, gt_data, plot_dim

  # Remove the main axes
  removeAxes(ax_gt)
  removeAxes(ax_odom)
  removeAxes(ax_vertices)

  # Ground truth.
  if (gt_data_path is not None and os.path.isfile(gt_data_path) and isFull(gt_data_path)):

    # Check gt file type.
    f = open(gt_data_path)
    lines = f.readlines()
    f.close()
    size = lines[1].split(",")
    if (len(size) > 12):
      try:
        data = np.loadtxt(gt_data_path, delimiter = ",", skiprows = 1, usecols = (0, 5, 6, 7, 8, 9, 10, 11))
      except Exception as e:
        print(f"The following exception was detected while trying to load the GT: {e} ")
        return
    else:
      try:
        data = np.loadtxt(gt_data_path, delimiter=',', skiprows = 1, usecols = (0, 1, 2, 3, 4, 5, 6, 7))
      except Exception as e:
        print(f"The following exception was detected while trying to load the GT: {e} ")
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
  if (os.path.isfile(odom_data_path) and isFull(odom_data_path)):

    # Read the data
    try:
      data = np.loadtxt(odom_data_path, delimiter = ",", skiprows = 1, usecols = (5, 6, 7, 8, 9, 10, 11))
      # data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(4, 5, 6, 7, 8, 9, 10))
    except:
      print("Problems loading odometry data.")
      return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_odom = ax.plot(data[:, 0], data[:, 1], data[:, 2], "blue", label = "Odometry")
    else:
      ax_odom = ax.plot(data[:, 0], data[:, 1], "blue", label = "Odometry")

  # Load SLAM vertices (file saved with SLAM node)
  if (os.path.isfile(slam_data_path) and isFull(slam_data_path)):

    # Check if file is blocked
    while (os.path.exists(lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = np.loadtxt(slam_data_path, delimiter = ",", skiprows = 1, usecols = (2, 3, 4, 5, 6, 7, 8))
    except Exception as e:
      print(f"The following exception was detected while trying to load the SLAM algorithm vertices: {e} ")
      return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_vertices = ax.plot(data[:, 0], data[:, 1], data[:, 2], "green", label = "SLAM", marker = "o", zorder = 1)
    else:
      ax_vertices = ax.plot(data[:, 0], data[:, 1], "green", label = "SLAM", marker = "o", zorder = 1)

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
  
  parser = argparse.ArgumentParser(
          description = "SLAM online viewer.",
          formatter_class = argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("-gt", "--gt", type = str, 
          help = "Path to the file containing the ground truth data (optional).",
          default = "/home/user/gt.txt")
  parser.add_argument("-o", "--o", type = str, 
          help = "Path to the file containing the odometry data (required).",
          default = "/home/user/odom.txt")
  parser.add_argument("-v", "--v", type = str, 
          help = "Path to the file containing the SLAM vertices data (required).",
          default = "/home/user/vertices.txt")
  parser.add_argument("-e", "--e", type = str, 
          help = "Path to the file containing the SLAM loop closing edges data (optional).",
          default = "/home/user/edges.txt")
  parser.add_argument("-dim", "--dim", type = int,
          help = "Defines the plot dimensions: 2 for xy and 3 for xyz.",
          default = 3)
  args = parser.parse_args()

  # Some hardcode parameters
  # font = {'family' : 'Sans',
  #         'weight' : 'normal',
  #         'size'   : 30}
  # pylab.rc('font', **font)

  # Sanity check
  if (not os.path.isfile(args.v) or not os.path.isfile(args.o)):
    sys.exit()

  print("GRAPH VIEWER MOUSE INPUTS:")
  print(" - Right button: activates/deactivates the visualization of graph edges.")

  # Save global variables
  plot_dim = args.dim
  lock_file = args.o + ".lock"
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
  plotOnline(args.gt, args.o, args.v)
  timer.add_callback(plotOnline, args.gt, args.o, args.v)
  timer.start()
  plt.show()
