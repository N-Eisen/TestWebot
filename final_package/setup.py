from setuptools import find_packages, setup

package_name = 'final_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/launch_01.py',
]))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/robotic_arms.wbt',
    'worlds/.robotic_arms.wbproj'
]))
data_files.append(('share/' + package_name, [
    'package.xml'
]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nicolai',
    maintainer_email='nicolai@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ur5e_controller = final_package.ur5e_controller:main'
        ],
    },
)
