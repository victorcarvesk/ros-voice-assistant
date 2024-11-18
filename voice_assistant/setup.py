from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'voice_assistant'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Victor Carvesk',
    maintainer_email='victorcarvesk@gmail.com',
    description='vortex NLP files',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mic_node = voice_assistant.mic_node:main',
            'nlp_node = voice_assistant.nlp_node:main',
            'speaker_node = voice_assistant.speaker_node:main',
        ],
    },
)
