from setuptools import setup, find_packages

def parse_requirements(filename):
    """Load requirements from a pip requirements file."""
    with open(filename, 'r') as file:
        return [line.strip() for line in file if line.strip() and not line.startswith('#')]

setup(
    name="mks_servo_rs485",
    version="0.1",
    packages=find_packages(),
    install_requires=parse_requirements('requirements.txt'),
    author="Marek Engelbrink",
    author_email="marek.engelbrink@gmail.com",
    description="A simple example private package",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    url="https://github.com/marekengelbrink/mks-servo-rs485",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: BSD License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.12',
)
