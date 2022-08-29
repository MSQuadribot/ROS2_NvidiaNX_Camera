from setuptools import setup

package_name = 'camera'

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
    maintainer='YourName',
    maintainer_email='you@email.com',
    description='camera publisher/subscriber using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'publisher = camera.camera_dualpub:main',
                'listener = camera.camera_listener:main',
                'result = camera.camera_result:main',
                'display = camera.camera_display:main',
                'posting = camera.camera_web:main',
                'streamhost = camera.camera_streamhost:main',
                'streamclient = camera.camera_streamclient:main',
        ],
    },
)
