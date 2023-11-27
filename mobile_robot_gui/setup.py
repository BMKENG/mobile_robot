from setuptools import find_packages, setup

package_name = 'mobile_robot_gui'

setup(
    # scripts=['mobile_robot_gui/mobile_robot_gui'],
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치파일 추가 
        ('share/' + package_name + '/launch', ['launch/mobile_robot_gui.launch.py']),
        ('share/' + package_name + '/launch', ['launch/mobile_robot_behavior_gui.launch.py']),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ggh',
    maintainer_email='0380089@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mobile_robot_gui = mobile_robot_gui.mobile_robot_gui:main',
            'mobile_robot_behavior_gui = mobile_robot_gui.mobile_robot_behavior_gui:main',
            'mobile_robot_behavior_client_gui = mobile_robot_gui.mobile_robot_behavior_client_gui:main',
        ],
    },
)
