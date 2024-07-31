from setuptools import find_packages, setup
import pathlib

package_name = 'knowledge_core'


def install_configuration(path):

    tmp = pathlib.Path("./marker")
    tmp.mkdir(exist_ok=True)
    with open(tmp/package_name, 'w') as f:
        f.write(path)

    return [
        (f'share/ament_index/resource_index/pal_configuration.{package_name}/', [
         str(tmp/package_name)]),
        (f'share/{package_name}/config', [path]),
    ]


def readme():
    with open("README-pypi.rst") as f:
        return f.read()


setup(
    name=package_name,
    version="3.2.3",
    license='Apache License 2.0',
    description="A RDFlib-backed knowledge base for robotic applications",
    long_description=readme(),
    packages=find_packages(where="src", exclude=['test']),
    package_dir={"": "src"},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/pal_system_module',
         ['module/' + package_name]),
        ('share/' + package_name + '/launch',
         ['launch/knowledge_core.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ] + install_configuration('config/00-default.yaml'),
    install_requires=['setuptools'],
    zip_safe=True,
    author="Séverin Lemaignan",
    author_email="severin.lemaignan@pal-robotics.com",
    maintainer='Séverin Lemaignan',
    maintainer_email='severin.lemaignan@pal-robotics.com',
    url="https://github.com/severin-lemaignan/knowledge_core",
    tests_require=['pytest'],
    scripts=['bin/knowledge_core'],
)
