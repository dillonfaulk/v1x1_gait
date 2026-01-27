from setuptools import find_packages, setup

package_name = 'v1x1_gait'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'ikpy>=3.4.2',
        'numpy>=1.26.4',
        'numba>=0.62.1',
        'scipy>=1.12',
        'pyyaml>=6.0.1'
    ],
    zip_safe=True,
    maintainer='n3z',
    maintainer_email='dillonfaulk92@gmail.com',
    description='V1X1 gait control package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # --- Gait Commander --- #
            'v1x1_gait_commander = v1x1_gait.v1x1_gait_commander:main',
            # --- Gait Engine --- #
            'v1x1_gait_engine = v1x1_gait.v1x1_gait_engine:main',
            # --- Stance Commander --- #
            'v1x1_stance_commander = v1x1_gait.v1x1_stance_commander:main'
        ],
    },
)
