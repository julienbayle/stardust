from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
        packages=['sd_led_matrix'],
        package_dir={'' : 'scripts'},
        license='MIT',
        install_requires=["luma.led_matrix"]
        )
                  
setup(**setup_args)
