#!/usr/bin/env python

from distutils.core import setup

try:
    from catkin_pkg.python_setup import generate_distutils_setup

    setup_args = generate_distutils_setup(
        packages=['phastapromp','labelapp'],
        package_dir={'phastapromp': 'src/phastapromp','labelapp': 'src/labelapp'},
         )
    setup(**setup_args)

except ImportError:
    setup(name='Helpers to use Phase-State Machines and Probabilistic Movement Primitives together',
          version='0.1.0',
          description='Library to teach probabilistic motion primitives, which are distributions over continous trajectories',
          author='Raphael Deimel, Jan Martin',
          author_email='raphael.deimel@tu-berlin.de',
          url='http://www.mti-engage.tu-berlin.de/',
          packages=['phastapromp','labelapp'],
          package_dir={'phastapromp': 'src/phastapromp','labelapp': 'src/labelapp'},
         )

