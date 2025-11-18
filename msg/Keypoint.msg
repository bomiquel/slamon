# @file Keypoint.msg
# @brief Personalised message to define a keypoint.
# @package slamon
# @project SLAMON
#
# @author
#   Bo Miquel Nordfeldt-Fiol (2024–2025)
#
# @license BSD-4-Clause
#   This file is part of the SLAMON project and is released under
#   the BSD 4-Clause License. See the LICENSE file for details.


# The keypoint descriptor
float32[] desc

# The keypoint image coordinates
geometry_msgs/Point pt