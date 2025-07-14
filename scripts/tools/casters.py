#!/usr/bin/env python3


"""
casters.py
==================

Type transformer.

Author
----------
Bo Miquel Nordfeldt-Fiol

Date
----------
2025
"""


import argparse


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