from setuptools import setup
import os
from glob import glob
package_name = 'yahboomcar_multi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'param'),glob(os.path.join('param','*.yaml'))),
        (os.path.join('share',package_name,'maps'),glob(os.path.join('maps','*.yaml'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*.launch.xml'))),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'queue = yahboomcar_multi.queue:main',
        'listenline = yahboomcar_multi.listenline:main',
        'yahboomcar_X3_ctrl = yahboomcar_multi.yahboom_X3_joy:main',
        'yahboomcar_R2_ctrl = yahboomcar_multi.yahboom_R2_joy:main'
        ],
    },
)
