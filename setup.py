from itertools import chain
import os

from setuptools import find_packages, setup

package_name = 'knowledge_core'


def generate_kb_explorer_files():
    data_files = []
    data_dirs = ('pages',)

    for path, _, files in chain.from_iterable(
            os.walk(data_dir) for data_dir in data_dirs):
        install_dir = os.path.join('share', package_name, path)
        list_entry = (install_dir, [os.path.join(path, f)
                      for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files


def readme():
    try:
        with open('README-pypi.rst') as f:
            return f.read()
    except FileNotFoundError:
        return ""


setup(
    name=package_name,
    version='3.11.0',
    license='Apache License 2.0',
    description='A RDFlib-backed knowledge base for robotic applications',
    long_description=readme(),
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/pal_system_module',
         ['module/' + package_name]),
        ('share/' + package_name + '/module', ['module/knowledge_core_module.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/knowledge_core.launch.py', 'launch/knowledge_viewer.launch.py']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/00-defaults.yaml']),
        ('share/' + package_name + '/config', ['config/knowledge_core_analyzers.yaml']),
        ('share/ament_index/resource_index/pal_configuration.' + package_name,
            ['config/' + package_name]),
    ] + generate_kb_explorer_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    author='Séverin Lemaignan',
    author_email='severin.lemaignan@pal-robotics.com',
    maintainer='Séverin Lemaignan',
    maintainer_email='severin.lemaignan@pal-robotics.com',
    url='https://github.com/severin-lemaignan/knowledge_core',
    tests_require=['pytest'],
    scripts=['bin/knowledge_core', 'bin/knowledge_viewer'],
)
