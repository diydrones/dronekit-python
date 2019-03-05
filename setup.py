from setuptools import find_packages, setup

version = '2.9.1'

try:
    with open(os.path.join(here, 'README.md'), encoding='utf-8') as f:
        LongDescription = f.read()
except FileNotFoundError:
    LongDescription = 'Python API for communication and control of drones over MAVLink.'


setup(
    name='dronekit',
    zip_safe=True,
    version=version,
    description='Developer Tools for Drones.',
    long_description=LongDescription,
    url='https://github.com/dronekit/dronekit-python',
    author='3D Robotics',
    install_requires=[
        'pymavlink>=2.2.20',
        'monotonic>=1.3',
    ],
    author_email='tim@3drobotics.com, kevinh@geeksville.com',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Topic :: Scientific/Engineering',
    ],
    license='apache',
    packages=find_packages()
)
