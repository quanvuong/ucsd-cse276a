import requests
import numpy as np
import time
from scipy import stats
from itertools import count
import sys

HOST      = '172.20.10.2'
PORT 	  = '8005'
autologin = 1

# BASE_URL is variant use to save the format of host and port
BASE_URL = 'http://' + HOST + ':'+ PORT + '/'

def __request__(url, times=10):
	for x in range(times):
		try:
			requests.get(url)
			return 0
		except :
			print("Connection error, try again")
	print("Abort")
	return -1

def run_action(cmd):
	"""Ask server to do sth, use in running mode

	Post requests to server, server will do what client want to do according to the url.
	This function for running mode

	Args:
		# ============== Back wheels =============
		'bwready' | 'forward' | 'backward' | 'stop'

		# ============== Front wheels =============
		'fwready' | 'fwleft' | 'fwright' |  'fwstraight'

		# ================ Camera =================
		'camready' | 'camleft' | 'camright' | 'camup' | 'camdown'
	"""
	# set the url include action information
	url = BASE_URL + 'run/?action=' + cmd
	print('url: %s'% url)
	# post request with url
	__request__(url)

def run_speed(speed):
	"""Ask server to set speed, use in running mode

	Post requests to server, server will set speed according to the url.
	This function for running mode.

	Args:
		'0'~'100'
	"""
	# Set set-speed url
	url = BASE_URL + 'run/?speed=' + speed
	print('url: %s'% url)
	# Set speed
	__request__(url)


def run_fwturn(angle):

	url = BASE_URL + 'run/?action=fwturn:' + str(angle)
	print('url: %s'% url)
	__request__(url)


def connection_ok():
	"""Check whetcher connection is ok

	Post a request to server, if connection ok, server will return http response 'ok'

	Args:
		none

	Returns:
		if connection ok, return True
		if connection not ok, return False

	Raises:
		none
	"""
	cmd = 'connection_test'
	url = BASE_URL + cmd
	print('url: %s'% url)
	# if server find there is 'connection_test' in request url, server will response 'Ok'
	try:
		r=requests.get(url)
		if r.text == 'OK':
			return True
	except:
		return False

# PHYSICAL SPEED - DIGITAL SPEED CALIBRATION
# run_speed('100')

# run_action('forward')
# time.sleep(5)
# run_action('stop')

# print('This moved 2.32 meters')
# print('Finished')

# run_speed('50')

# run_action('forward')
# time.sleep(5)
# run_action('stop')

# print('This moved 90 cm')
# print('Finished')

# run_speed('75')

# run_action('forward')
# time.sleep(5)
# run_action('stop')

# print('This moved 1.75 meters')
# print('Finished')

# run_speed('25')

# run_action('forward')
# time.sleep(5)
# run_action('stop')

# print('This moved 23 cm')
# print('Finished')

dig_speeds = np.array([25, 50, 75, 100])
dis_moved = np.array([0.23, 0.9, 1.75, 2.32])

phy_speeds = dis_moved / 5.0  # meter per second

# plt.plot(dig_speeds, phy_speeds)

slope, intercept, r_value, p_value, std_err = stats.linregress(phy_speeds, dig_speeds)

print(f'slope: {slope}')
print(f'intercept: {intercept}')

def phy_to_dig_speed(phy_speed):
    return phy_speed * slope + intercept

def dig_to_phy_speed(dig_speed):
    return (dig_speed - intercept) / slope

# for v in phy_speeds:
#     print(phy_to_dig_speed(v))

# PHYSICAL STEERING ANGLE TO DIGITAL STEERING ANGLE CALIBRATION

def phy_to_dig_steering_angle(angle):

    dis_from_straight_line = 7.0 * np.tan(abs(angle))

    # 7 is the distance between the intersection of the 2 lines coming from the two wheels positions
    # at the 2 extreme left and right steering position

    if angle < 0:  # turn right
        return 90 + dis_from_straight_line  # 90 is the angle to input into the car to put front wheel into forward heading
    elif angle > 0:  # turn left
        return 90 - dis_from_straight_line
    else:
        return 90


class BicycleKinematicModel(object):
    def __init__(self, x, y, L, fixed_timestep):
        self.theta = 0

        print(f'setting x to {x} and y to {y} in kinematics model')

        self.x = x
        self.y = y

        self.L = L  # wheel base
        self.fixed_timestep = fixed_timestep

    def forward(self, v, gamma):
        # Implement kinematic model

        print('---')
        print('in side kinematics forward')
        print('before forward pass')
        print(f'x: {self.x}')
        print(f'y: {self.y}')
        print(f'theta: {self.theta}')

        self.x += self.fixed_timestep * (v * np.cos(self.theta))
        self.y += self.fixed_timestep * (v * np.sin(self.theta))
        self.theta += self.fixed_timestep * (v * np.tan(gamma) / self.L)

        print('after forward pass')
        print(f'x: {self.x}')
        print(f'y: {self.y}')
        print(f'theta: {self.theta}')
        print('end of kinematics forward')
        print('---')

        return self.x, self.y, self.theta


def to_polar(x, y, theta, direction):

    rho = np.sqrt(x ** 2 + y ** 2)

    if direction == 0: # First time in simulation, the direction is not set
        beta = - np.arctan2(-y, -x)
        alpha = - theta - beta

        print('beta', beta)
        print('alpha', alpha)

        if (alpha > (np.pi/2.0 + 1e-4) or alpha < -(np.pi/2 + 1e-4)):
            # need to go backward
            direction = -1
        else:
            direction = 1
    elif direction == -1:
        beta = - np.arctan2(y, x)
        alpha = - theta - beta
    else:
        beta = - np.arctan2(-y, -x)
        alpha = - theta - beta

    # Set limit on alpha
    if alpha > np.pi/2.0:
        alpha = np.pi/2.0

    if alpha < - np.pi/2.0:
        alpha = - np.pi/2.0

    return rho, alpha, beta, direction


def we_are_there(rho):
    return np.abs(rho) < 0.1


def drive_pose(rho, alpha, beta, direction, ki_model, consts={}):

    kalpha = consts['kalpha']
    kbeta = consts['kbeta']
    krho = consts['krho']
    target = consts['target']
    wheel_base = consts['wheel_base']

    print(f'begin of drive pose. rho: {rho}, alpha: {alpha}, beta: {beta}, dir: {direction}')

    z1 = kalpha * alpha + kbeta * beta

    v = rho * krho * direction

    z2 = z1 * direction * wheel_base / np.abs(v)

    gamma = np.arctan(z2)

    # enforce physical constraint
    v = np.clip(v, -dig_to_phy_speed(100), dig_to_phy_speed(100))
    gamma = np.clip(gamma, -np.radians(90), np.radians(90))
    # Finished enforcing physical constraint

    x, y, theta = ki_model.forward(v, gamma)

    # change of coordinate
    x = x - target[0]
    y = y - target[1]

    rho, alpha, beta, direction = to_polar(x, y, theta, direction)

    print(f'end of drive pose. rho: {rho}, alpha: {alpha}, beta: {beta}, dir: {direction}')

    there_yet = we_are_there(rho)

    # change of coordinate
    beta = beta + target[2]

    return rho, alpha, beta, direction, v, gamma, there_yet

def test(target, hyperparams, debug=False):

    print(f'target: {target}')

    fixed_timestep = hyperparams['fixed_timestep']
    wheel_base = 0.15

    ki_model = BicycleKinematicModel(0.0, 0.0, wheel_base, fixed_timestep=fixed_timestep)  # the wheel base of the car is 15cm

    # change of coordinate for x, y
    x = 0.0 - target[0]
    y = 0.0 - target[1]

    consts = {
        'kalpha': 5,
        'kbeta': -2,
        'krho': 1,
        'target': target,
        'wheel_base': wheel_base
    }

    direction = hyperparams.get('direction', 0.0)

    rho, alpha, beta, direction = to_polar(x, y, 0.0, direction)

    beta = beta + target[2]

    print(f'rho: {rho}, alpha: {alpha}, beta: {beta}, dir: {direction}')

    start_t = time.time()

    # return

    for t in count():

        print()
        print(f'iteration: {t}')

        rho, alpha, beta, direction, v, gamma, there_yet = drive_pose(rho, alpha, beta, direction, ki_model, consts)

        dig_angle = int(phy_to_dig_steering_angle(gamma))
        dig_speed = int(phy_to_dig_speed(abs(v)))

        if not debug:
            run_fwturn(dig_angle)
            run_speed(str(dig_speed))

            if direction == 1:
                run_action('forward')
            elif direction == -1:
                run_action('backward')
            else:
                assert False, 'unrecognized directionn'

        print(f'angle. gamma: {gamma}, dig_angle: {dig_angle}')

        print(f'speed. v: {v}, dig_speed: {dig_speed}')


        time.sleep(fixed_timestep)

        if there_yet:
            print(f'We are there at: {target}')

            if not debug:
                run_action('stop')

            break

        if time.time() - start_t > hyperparams['time_limit']:
            print('Exceed time limit')

            if not debug:
                run_action('stop')

            break


def reset_car_state():
    run_action('fwready')
    run_action('bwready')
    run_action('camready')


if __name__ == '__main__':

    time.sleep(7)

    conn_ok = {connection_ok()}
    print(f'Connection Ok: {conn_ok}')

    if not conn_ok:
        sys.exit(1)

    # 1 to 2
    hyperparams = {
        'time_limit': 10,
        'fixed_timestep': 0.01
    }

    reset_car_state()
    test([0.15, 0, 0], hyperparams)  # this one works. woohoo.

    # 2 to 3
    hyperparams = {
        'time_limit': 3.5,
        'fixed_timestep': 0.01
    }

    reset_car_state()
    test([0.15, 0.15, np.radians(90)], hyperparams)

    # 3 to 4
    hyperparams = {
        'time_limit': 5,
        'fixed_timestep': 0.04,
    }

    reset_car_state()
    # test([0, -0.5, np.radians(45)], hyperparams, debug=True)
    test([0, -0.5, np.radians(45)], hyperparams)

    # 4 to 5
    reset_car_state()
    hyperparams = {
        'time_limit': 5,
        'fixed_timestep': 0.04,
        'direction': -1
    }
    test([0, 0.3, -np.radians(120)], hyperparams)
    #
    # 5 to 3
    reset_car_state()
    hyperparams = {
        'time_limit': 6,
        'fixed_timestep': 0.01
    }
    test([0.30, 0.1, np.radians(45)], hyperparams)

    # 3 to 1
    reset_car_state()
    hyperparams = {
        'time_limit': 6,
        'fixed_timestep': 0.01
    }
    test([0.20, -0.20, -np.radians(90)], hyperparams)

    run_action('stop')
    reset_car_state()
