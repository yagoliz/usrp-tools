project('usrp-tools', 'cpp', version: '0.0', default_options : ['c_std=c11', 'cpp_std=c++11'])

# Boost and UHD libraries
boost_dep = dependency('boost', modules : ['system','program_options'])
uhd_dep = dependency('uhd')

# Our precious executables
executable('query_gpsdo_sensors', 'src/query_gpsdo_sensors.cc', dependencies:[boost_dep, uhd_dep], install:true)
executable('sync_to_gps', 'src/sync_to_gps.cc', dependencies:[boost_dep, uhd_dep], install:true)
executable('rx_timed_samples_to_file', 'src/rx_timed_samples_to_file.cc', dependencies:[boost_dep, uhd_dep], install:true)
executable('rx_timed_samples_gpsdo', 'src/rx_timed_samples_gpsdo.cc', dependencies:[boost_dep, uhd_dep], install:true)
