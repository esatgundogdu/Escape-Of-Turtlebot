from setuptools import setup

package_name = 'assignment1'

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
    maintainer='humanoids-ubuntu',
    maintainer_email='humanoids-ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a1_referee = assignment1.a1_referee:main',
            'a1_answer = assignment1.a1_answer:main',
        ],
    },
)
