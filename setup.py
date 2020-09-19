from setuptools import setup

setup(
   name='Py3DR',
   version='0.1.0',
   author='HerbHSSO',
   description='A simple package for 3D Reconstruction',
   packages=['Py3DR'],
   long_description=open('README.txt').read(),
   install_requires=[
       "opencv-contrib-python",
       "open3d",
       "numpy",
   ],
)
