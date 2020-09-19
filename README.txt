Copyright Py3DR®

================================
Tutorial:
================================

import Py3DR

#Generate 3D Points Clouds and Create file.ply
variable_reconstruction3D = Py3DR.reconstruction3D('imageLeft.jpg', 'imageRight.jpg', 'output_3d.ply')

#Read the file.ply
variable_read_ply = Py3DR.write_plt('output_3d.ply')





Credits: OpenCV