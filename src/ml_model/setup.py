from setuptools import find_packages, setup

package_name = 'ml_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gramisaam',
    maintainer_email='gramisaam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['model_node = ml_model.model_node:main',
                            'ambulance_controller = ml_model.ambulance_controller:main'
        ],
    },
)
