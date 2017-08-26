
tuning = {
	'PID_xy': [5,0,0],
	'PID_z': [0,0,0],
	'NONLIN_SAFETY': True,
	'MAX_SAFE_ANGLE': 5,
}

cfg = {
    'port': '/dev/ttyS0',
    'baud': 115200,
    'UDP_IP': '0.0.0.0',
    'UDP_PORT': 9876,
    'wait_ready' : False,
    'actuatorPeriod': 0.05,
    'guidancePeriod': 0.01,
    'takeoff_altitude': 0.5,
    'verbose': 1,   # 0,1,2
	'tuning': tuning
    }
