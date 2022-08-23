from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

"""
Python setup file


"""


a = generate_distutils_setup(packages = ['lqr'],package_dir={'':'src'})
setup(**a)