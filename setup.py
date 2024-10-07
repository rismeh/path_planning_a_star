from setuptools import setup

package_name = 'smec_path_planning'

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
    maintainer='jua3316s@hs-coburg.de',
    maintainer_email='jua331s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pathpl=smec_path_planning.smec_path:main',
        ],
    },
)
