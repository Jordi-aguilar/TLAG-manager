from setuptools import setup, find_packages

setup(
    name='tlag_experiment',
    version='0.1',
    packages=['tlag_experiment'],
    entry_points={
        'console_scripts': [
            'tlag_cli = scripts.tango.cli_experiment_tango:main',
        ],
    },
)
