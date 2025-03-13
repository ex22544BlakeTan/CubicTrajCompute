from setuptools import find_packages, setup
import os
package_name = 'ar_interface'

setup(
    name=package_name,
    version='0.0.0',
    #packages=[find_packages(exclude=['test'])],
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share/', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'msg'),['msg/CubicTrajParams.msg']),
        (os.path.join('share', package_name, 'msg'),['msg/CubicTrajCoeffs.msg']),
        (os.path.join('share', package_name, 'srv'),['srv/ComputeCubicTraj.srv']),
        #('share/' + package_name + '/msg', ['msg/cubic_traj_coeffs.msg']),
        #('share/' + package_name + '/srv', ['srv/compute_cubic_traj.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ex22544',
    maintainer_email='ex22544@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0 ',
    #tests_require=['pytest'],
    #entry_points={
        #'rosidl_generate_interfaces':[
        #'ar_interface/msg/CubicTrajParams.msg'
        #'ar_interface/msg/CubicTrajCoeffs.msg'
        #'ar_interface/srv/ComputeCubicTraj.srv']
    #},
)
