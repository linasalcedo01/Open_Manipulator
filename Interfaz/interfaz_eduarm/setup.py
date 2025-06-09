## setup.py en ~/catkin_ws/src/interfaz_eduarm/
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['interfaz_eduarm'],
    package_dir={'': 'scripts'}
)

setup(**d)
