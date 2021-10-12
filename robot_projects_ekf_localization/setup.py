from setuptools import setup
from os import listdir

package_name = 'robot_projects_ekf_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/' + x for x in listdir('launch') if ".launch.py" in x]),
        ('share/' + 'config', ['config/ekf_sim_world.rviz', 'config/ekf_evaluate_params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nick',
    maintainer_email='nickm@outofthisworld.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_localization = robot_projects_ekf_localization.ekf_localization_node:main',
            'ekf_evaluation = robot_projects_ekf_localization.ekf_evaluation_node:main'
        ],
    },
)
