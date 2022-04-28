from setuptools import setup, find_packages, Extension
from Cython.Build import cythonize

ext_modules = []

ext_modules.append(Extension(
    'walkthrough.cbigproduct',
    sources=['walkthrough/cbigproductmodule.c']))

ext_modules.extend(cythonize('cython_interface.pyx'))

setup(
    name='cython_interface',
    packages=find_packages(),
    ext_modules=ext_modules,
)
