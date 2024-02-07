from setuptools import find_packages, setup

package_name = 'knowledge_core'

def readme():
    with open("README-pypi.rst") as f:
        return f.read()


setup(
    name=package_name,
    version="2.9.0",
    license='Apache-2.0',
    description="A RDFlib-backed simple knowledge based for robotic application",
    long_description=readme(),
    packages=find_packages(where="src", exclude=['test']),
    package_dir={"": "src"},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
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
