#!/usr/bin/env python3


"""
file_helpers.py
==================

Functions for processing files.

Author
----------
Bo Miquel Nordfeldt-Fiol

Date
----------
2025
"""


import os


def checkPathExistence(path_to_file, label):
  r"""
  Check if the path exists.
  """
  if path_to_file:
    if not os.path.isfile(path_to_file):
      print(f"{label} is not a file")
      return False
  else:
    print(f"{label} is None")
    return False
  
  return True


def checkFileType(path_to_file):

  with open(path_to_file, "r") as file:
      lines = file.readlines()
  size = lines[1].split(",")
  if (len(size) > 12):
      cols = (0, 5, 6, 7)
      symbol = ">"
      time_in_seconds = False
  else:
      cols = (0, 2, 3, 4)
      symbol = "<="
      time_in_seconds = True

  return cols, symbol, time_in_seconds


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