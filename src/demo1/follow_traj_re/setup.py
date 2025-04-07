from setuptools import setup

package_name = 'follow_traj_re'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hcx',
    maintainer_email='hcx@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test' : ['pytest'],
    },
    entry_points={
        'console_scripts': [
         'can_node  = follow_traj_re.can_node:main',
         'follow_node  = follow_traj_re.mpc_node:main',
         'follow_node_v2  = follow_traj_re.mpc_node_v2:main',
         'change_node = follow_traj_re.change_node:main',
        ],
    },
)
