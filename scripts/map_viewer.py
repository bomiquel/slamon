#!/usr/bin/env python3

"""
@file
@brief Real-time map viewer.
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
gt_data = []
ax_odom = None
ax_edges = []
lock_file = ""
ax_vertices = None
edges_shown = True
graph_edges_path = ""
legend_edited = False


class Error(Exception):
  """ Base class for exceptions in this module. """
  pass


def file_is_full(path_to_file):
  """ Check if the file length is > 0 """
  f = open(path_to_file)
  lines = f.readlines()
  f.close()
  return len(lines) > 0


def rm_ax(ax_id):
  """ Remove the axes lines """
  if (ax_id is not None and ax_id):
    l = ax_id.pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l


def real_time_plot(gt_path, odom_path, graph_vertices_path):
  """ Function to plot the data saved into the files in real time. """

  global lock_file, legend_edited, ax_gt, ax_odom, ax_vertices, edges_shown, gt_data, plot_dim

  # Remove the main axes
  rm_ax(ax_gt)
  rm_ax(ax_odom)
  rm_ax(ax_vertices)

  # Ground truth.
  if (gt_path is not None and os.path.isfile(gt_path) and file_is_full(gt_path)):

    # Check gt file type.
    f = open(gt_path)
    lines = f.readlines()
    f.close()
    size = lines[1].split(",")
    if (len(size) > 12):
      try:
        data = np.loadtxt(gt_path, delimiter = ",", skiprows = 1, usecols = (0, 5, 6, 7, 8, 9, 10, 11))
      except Exception as e:
        print(f"The following exception was detected while trying to load the GT: {e} ")
        return
    else:
      try:
        data = np.loadtxt(gt_path, delimiter=',', skiprows = 1, usecols = (0, 1, 2, 3, 4, 5, 6, 7))
      except Exception as e:
        print(f"The following exception was detected while trying to load the GT: {e} ")
        return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])
  
    # Plot.
    if (plot_dim == 3):
      ax_gt = ax.plot(data[:, 1], data[:, 2], data[:, 3], "k", label = "Ground Truth")
    else:
      ax_gt = ax.plot(data[:, 1], data[:, 2], "k", label = "Ground Truth")

  # Load visual odometry data (saved with rostopic echo -p /stereo_odometer/odometry > file.txt).
  if (os.path.isfile(odom_path) and file_is_full(odom_path)):

    # Read the data
    try:
      data = np.loadtxt(odom_path, delimiter = ",", skiprows = 1, usecols = (5, 6, 7, 8, 9, 10, 11))
      # data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(4, 5, 6, 7, 8, 9, 10))
    except:
      print("Problems loading odometry data.")
      return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_odom = ax.plot(data[:, 0], data[:, 1], data[:, 2], "c", label = "Odometry")
    else:
      ax_odom = ax.plot(data[:, 0], data[:, 1], "c", label = "Odometry")

  # Load stereo slam vertices (file saved with node slamon)
  if (os.path.isfile(graph_vertices_path) and file_is_full(graph_vertices_path)):

    # Check if file is blocked
    while (os.path.exists(lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = np.loadtxt(graph_vertices_path, delimiter = ",", skiprows = 1, usecols = (2, 3, 4, 5, 6, 7, 8))
    except Exception as e:
      print(f"The following exception was detected while trying to load the SLAM algorithm vertices: {e} ")
      return

    # Pre-process plot if necessary.
    if (len(data.shape) == 1):
      data = np.array([data])

    # Plot.
    if (plot_dim == 3):
      ax_vertices = ax.plot(data[:, 0], data[:, 1], data[:, 2], "b", label = "SLAM", marker = "o", zorder = 1)
    else:
      ax_vertices = ax.plot(data[:, 0], data[:, 1], "b", label = "SLAM", marker = "o", zorder = 1)

  # Show the edges
  if (edges_shown == True):
    draw_edges()
  else:
    remove_edges()

  # Update the plot
  plt.draw()

  # Show legend only once
  if (ax_vertices is not None and legend_edited is False):
    ax.legend()
    legend_edited = True


def draw_edges():
  """ Draw the edges """
  global lock_file, graph_edges_path, ax_edges, edges_shown, plot_dim

  # First, remove previous edges.
  remove_edges()

  # Load stereo slam edges (file saved with node stereo_slam)
  if (os.path.isfile(graph_edges_path) and file_is_full(graph_edges_path)):

    # Check if file is blocked
    while (os.path.exists(lock_file)):
      time.sleep(0.5)

    # Read the data
    try:
      data = np.loadtxt(graph_edges_path, delimiter = ",", skiprows = 1, usecols = (0, 1, 2, 3, 4, 9, 10, 11))
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
        color = "r"
      else:
        color = "b"
      vect = []
      vect.append([data[i, 2], data[i, 3], data[i, 4]])
      vect.append([data[i, 5], data[i, 6], data[i, 7]])
      vect =  np.array(vect)
      if (plot_dim == 3):
        ax_edge = ax.plot(vect[:, 0], vect[:, 1], vect[:, 2], color, linestyle = "-", zorder = 1)
      else:
        ax_edge = ax.plot(vect[:, 0], vect[:, 1], color, linestyle = "-", zorder = 1)
      ax_edges.append(ax_edge)

  edges_shown = True

  return


def remove_edges():
  """ Remove the edges """
  global ax_edges, edges_shown

  for i in range(len(ax_edges)):
    l = ax_edges[i].pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l
  ax_edges = []
  edges_shown = False


def onclick(event):
  """ Handle the click event """
  global edges_shown

  if (event.button == 3):
    if (edges_shown):
      remove_edges()
    else:
      draw_edges()
    plt.draw()


if __name__ == "__main__":
  
  parser = argparse.ArgumentParser(
          description = "Plot in 2D or 3D graphics of odometry data files in online.",
          formatter_class = argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument("-d", "--d", type = str, 
          help = "directory with the files: gt.txt (optional), odom.txt, graph_vertices.txt, graph_edges.txt",
          default = "")
  parser.add_argument("-gt", "--gt", type = str, 
          help = "file with ground truth",
          default = "")
  parser.add_argument("-o", "--o", type = str, 
          help = "file with visual odometry",
          default = "")
  parser.add_argument("-v", "--v", type = str, 
          help = "file with the vertices of stereo slam",
          default = "")
  parser.add_argument("-e", "--e", type = str, 
          help = "file with the edges of slamon",
          default = "")
  parser.add_argument("-dim", "--dim", type = int,
          help = "defines the plot dimensions: 2 for xy and 3 for xyz",
          default = 3)
  args = parser.parse_args()
  plot_dim = args.dim

  # Some hardcode parameters
  # font = {'family' : 'Sans',
  #         'weight' : 'normal',
  #         'size'   : 30}
  # pylab.rc('font', **font)

  print("GRAPH VIEWER MOUSE INPUTS:")
  print(" - Right button: activates/deactivates the visualization of graph edges.")

  # Default parameters
  if(args.d != ""):

    if os.path.exists(args.d):
      visual_odometry_path = os.path.join(args.d, args.o)
      graph_vertices_path = os.path.join(args.d, args.v)
      graph_edges_path = os.path.join(args.d, args.e)

      if args.gt != "":
        ground_truth_path = os.path.join(args.d, args.gt)
      else:
        ground_truth_path = None

    else:
      print("The path to the working directory doesn't exist. Exiting.")
      sys.exit()

  else:
    print("You have not inserted the path to the working directory. Exiting.")
    sys.exit()

  # Save blocking file into global
  lock_file = graph_vertices_path + ".lock"

  # Init figure
  fig = plt.figure(1)
  if (plot_dim == 3):
    ax = fig.add_subplot(111, projection = '3d')
    ax.set_zlabel("z (m)")
  else:
    ax = fig.add_subplot(111)
    ax.grid()

  ax.set_title("Map Viewer")
  ax.set_xlabel("x (m)")
  ax.set_ylabel("y (m)")

  # Handle on click callback
  fig.canvas.mpl_connect('button_press_event', onclick)

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  real_time_plot(ground_truth_path, visual_odometry_path, graph_vertices_path)
  timer.add_callback(real_time_plot,
                     ground_truth_path,
                     visual_odometry_path,
                     graph_vertices_path)
  timer.start()
  plt.show()
