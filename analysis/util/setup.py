from setuptools import setup

# usage
# python setup.py develop --user

setup(
    name='octopus_gps',
    version='1.5.0',
    author='Zhibei Ma',
    classifiers=['Private :: Do Not Upload'],
    install_requires=['numpy'],
    packages=['llh2enu']
)
