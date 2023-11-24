from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 패키지 정보를 설정합니다
setup_args = generate_distutils_setup(
    packages=['ur3_moveit_pkg'],
    package_dir={'': 'src'},
)

setup(**setup_args)
