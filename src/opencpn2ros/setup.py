from setuptools import setup
package_name = 'opencpn2ros'
setup(name=package_name, version='2.0.0', packages=[package_name], data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name + '/launch', ['launch/nmea_parser.launch.py']), ('share/' + package_name, ['package.xml'])], install_requires=['setuptools'], zip_safe=True, entry_points={'console_scripts': ['ais_parser = opencpn2ros.nodes:ais_main', 'arpa_parser = opencpn2ros.nodes:arpa_main', 'nmea_udp_bridge = opencpn2ros.nodes:nmea_main']})
