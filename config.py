
tuning = {
	#'PID_xy': [4,0,2],
	'PID_xy': [10,0,5],
	'PID_z': [0.1,0,0.1],
	'NONLIN_SAFETY': False,
	'MAX_SAFE_ANGLE': 4,
}

cfg = {
    'port': '/dev/ttyS0',
    'baud': 115200,
    'UDP_IP': '0.0.0.0',
    'UDP_PORT': 9876,
    'wait_ready' : False,
    'actuatorPeriod': 0.1,
    'guidancePeriod': 0.1,
    'estimatorPeriod': 0.02,
    'takeoff_altitude': 0.5,
    'verbose': 1,   # 0,1,2
	'tuning': tuning,
	'VICON_DRONENAME': 'BURAK_DRONE'
    }
