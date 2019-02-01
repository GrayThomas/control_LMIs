""" Install the control_LMIs package """
from distutils.core import setup
setup(
    name="control_LMIs",
    packages=["control_LMIs"],
    version="0.0.0",
    description="control library extension to support LMIs",
    author="Gray Thomas",
    author_email="gray.c.thomas@gmail.com",
    url="https://graythomas.github.io/control_LMIs",
    download_url="https://graythomas.github.io/download/python3-control_LMIs-0.0.0.tgz",
    keywords=["control", "LMI", "convex optimization"],
    classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.6",
        "Development Status :: 1 - Planning",
        # "Development Status :: 2 - Pre-Alpha",
        # "Development Status :: 3 - Alpha",
        # "Development Status :: 4 - Beta",
        "Environment :: Console",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Natural Language :: English",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering",
        "Topic :: Scientific/Engineering :: Mathematics",
        "Topic :: Software Development :: Libraries :: Python Modules",
        "Topic :: Utilities"
        ],
    long_description="""\
Feedback Control System Design Linear Matrix Inequalities (LMI) Utility 
-----------------------------------------------------------------------

Object oriernted framework for manipulating systems and setting up LMI-format control synthesis problems.


This version requires Python 3 or later.

"""
)
