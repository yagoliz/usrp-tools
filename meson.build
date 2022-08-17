project('usrp', 'cpp', version: '0.0')

boost_dep = dependency('boost', modules : ['program_options'])
uhd_dep = dependency('uhd')

executable('query_gpsdo_sensors', 'src/query_gpsdo_sensors.cc', dependencies:[boost_dep, uhd_dep])
executable('sync_to_gps', 'src/sync_to_gps.cc', dependencies:[boost_dep, uhd_dep])
executable('rx_timed_samples_to_file', 'src/rx_timed_samples_to_file.cc', dependencies:[boost_dep, uhd_dep])
executable('rx_timed_samples_gpsdo', 'src/rx_timed_samples_gpsdo.cc', dependencies:[boost_dep, uhd_dep])