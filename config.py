
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
    'actuatorPeriod': 0.2,
    'guidancePeriod': 0.2,
    'estimatorPeriod': 0.2,
    'takeoff_altitude': 0.5,
    'verbose': 2,   # 0,1,2
	'tuning': tuning,
	'VICON_DRONENAME': 'BURAK_DRONE',
	
	'btAddr': '00:16:D4:F3:59:70',
	'btUUID': '00001101-0000-1000-8000-00805F9B34FB',

	'EstimatorScheme': 'ZeroEstimator',
	'ActuatorScheme': 'SimpleVelocityController',
	'GuidanceScheme': 'MissionGuidance'
    }
