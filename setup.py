from setuptools import setup, find_packages
import codecs
import os

here = os.path.abspath(os.path.dirname(__file__))

with codecs.open(os.path.join(here, "README.md"), encoding="utf-8") as fh:
    long_description = "\n" + fh.read()

VERSION = '0.0.1'
DESCRIPTION = 'Utility package for QoL improvements, plotting, and robotics'

# Setting up
setup(
    name="mf-utils",
    version=VERSION,
    author="Mads Frier",
    author_email="madsfrier@gmail.com",
    description=DESCRIPTION,
    packages=find_packages(),
    install_requires=['matplotlib', 'numpy'],
    keywords=['python', 'utility', 'robotics', 'qol'],
    classifiers=[]
)