#!/usr/bin/env python
# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

__author__ = 'Kai Zhou'

setup(
    name='vsimple',
    version='1.0.2',
    author='Kai Zhou',
    author_email='kai.zhou@tusimple.ai',
    description='Vsimple',
    url='https://github.com/TuSimple/vsimple2.git',
    # data_files=[('kuguanio/config', ['kuguanio/config/config.txt']),
    #             ('kuguanio/config/compress', ['kuguanio/config/compress/object_compress.json'])],
    packages=find_packages()
)