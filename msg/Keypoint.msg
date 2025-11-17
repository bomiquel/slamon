# @file Keypoint.msg
# @brief Personalised message to define a keypoint.
# @package slamon
#
# @author
#        Bo Miquel Nordfeldt-Fiol (2024–2025)
#
# @copyright
#        Copyright (c) 2025 Bo Miquel Nordfeldt-Fiol
#
# @license BSD-3-Clause
#        This software is released under the BSD 3-Clause License.
#        See the LICENSE file for full text.

# The keypoint descriptor
float32[] desc

# The keypoint image coordinates
geometry_msgs/Point pt