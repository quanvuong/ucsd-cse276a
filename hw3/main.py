import requests
import numpy as np
import time
from scipy import stats
import http

from pyzbar import pyzbar
import imutils
import cv2
import scipy
import pickle


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
	# print('url: %s'% url)
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
	# print('url: %s'% url)
	# Set speed
	__request__(url)


def run_fwturn(angle):

	url = BASE_URL + 'run/?action=fwturn:' + str(angle)
	# print('url: %s'% url)
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
        print(f'x: {self.x}')
        print(f'y: {self.y}')
        print(f'theta: {self.theta}')

        self.x += self.fixed_timestep * (v * np.cos(self.theta))
        self.y += self.fixed_timestep * (v * np.sin(self.theta))
        self.theta += self.fixed_timestep * (v * np.tan(gamma) / self.L)

        print(f'x: {self.x}')
        print(f'y: {self.y}')
        print(f'theta: {self.theta}')

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


class QueryImage(object):
    """Query Image

    Query images form http. eg: queryImage = QueryImage(HOST)

    Attributes:
        host, port. Port default 8080, post need to set when creat a new object

    """
    def __init__(self, host, port=8080, argv="/?action=snapshot"):
        # default port 8080, the same as mjpg-streamer server
        self.host = host
        self.port = port
        self.argv = argv

    def queryImageAsByte(self):
        """Query Image

        Query images form http.eg:data = queryImage.queryImage()

        Args:
            None

        Return:
            returnmsg.read(), http response data
        """
        http_data = http.client.HTTPConnection(self.host, self.port)
        http_data.putrequest('GET', self.argv)
        http_data.putheader('Host', self.host)
        http_data.putheader('User-agent', 'python-http.client')
        http_data.putheader('Content-type', 'image/jpeg')
        http_data.endheaders()
        returnmsg = http_data.getresponse()

        return returnmsg.read()

    def getBarCodes(self):

        img_as_byte = self.queryImageAsByte()

        img = cv2.imdecode(np.fromstring(img_as_byte, np.uint8), cv2.IMREAD_GRAYSCALE)

        img = imutils.resize(img, width=400)

        barcodes = pyzbar.decode(img)

        return barcodes, img


def reset_car_state():
    run_action('fwready')
    run_action('bwready')
    run_action('camready')


def drive_to_pose(speed, num_loop):

    prev_angle = None

    for _ in range(num_loop):

        print()

        query_image = QueryImage(HOST)
        barcodes, img = query_image.getBarCodes()

        if not barcodes:
            print('Did not find barcodes')

            if prev_angle is None:
                continue

            if prev_angle < 90:
                dig_angle = 90 + (90 - prev_angle)
            else:
                dig_angle = 90 - (prev_angle - 90)

            run_fwturn(dig_angle)

            continue

        barcode = barcodes[0]

        frame_center = (150, 200)

        (x, y, w, h) = barcode.rect

        print(f'x: {x}, y: {y}, w: {w}, {h}')

        center = (x + w/2.0, y + h/2.0)

        dis = center[0] - frame_center[0], center[1] - frame_center[1]

        if dis[0] > 0:
            steering_angle = 0.0 - 0.5 * dis[0]

        else:
            steering_angle = 0.5 * dis[0]

        print(f'dis: {dis}, steering angle: {steering_angle}')

        dig_angle = int(phy_to_dig_steering_angle(steering_angle))

        dig_angle = np.clip(dig_angle, 81, 99)

        prev_angle = dig_angle

        run_fwturn(dig_angle)
        run_speed(str(speed))
        run_action('forward')

        time.sleep(0.1)

    run_action('stop')


def drive_in_circle(speed, angle, run_time=1.0):
    dig_angle = int(phy_to_dig_steering_angle(angle))

    run_fwturn(dig_angle)
    run_speed(str(speed))
    run_action('forward')

    time.sleep(run_time)

    run_action('stop')


def hard_code_odo():

    return 0.1, np.radians(5)


def get_next_pose_from_odo(x, odo):
    "Adapted from https://github.com/petercorke/robotics-toolbox-matlab/blob/0782e31c523f2bc94c259c2039f9ce4911c48533/Vehicle.m#L200"
    dd = odo[0]
    dth = odo[1]

    xnext = [0, 0, 0]

    thp = xnext[2] = (x[2] + dth) % np.pi

    xnext[0] = x[0] + dd * np.cos(thp)
    xnext[1] = x[1] + dd * np.sin(thp)

    return np.array(xnext)


def get_Fx(x, odo):
    "Adapted from https://github.com/petercorke/robotics-toolbox-matlab/blob/0782e31c523f2bc94c259c2039f9ce4911c48533/Vehicle.m#L250"

    dd = odo[0]
    thp = x[2] + odo[1]

    return np.array([
        [1, 0, -dd * np.sin(thp)],
        [0, 1, dd * np.cos(thp)],
        [0, 0, 1]
    ])


def get_bar_code():

    query_image = QueryImage(HOST)
    barcodes, img = query_image.getBarCodes()

    if not barcodes:
        return []

    barcode = barcodes[0]

    # we assume we don't see 2 barcode together
    if len(barcodes) > 2:
        print(f'More than one barcode found: {barcodes}')
        return barcodes[0]

    return barcode


def get_barcode_idx(barcode_name, detected_at_pose,
                    barcode_idx_to_name, barcode_idx_detected_at_post):
    "Data association"

    for bc_idx, val in barcode_idx_to_name.items():
        # bc_idx is the idx of the barcode
        # val is bar code name
        # The bar code name can be QR 3 or something similar to it

        if val == barcode_name:
            bc_detected_at_pose = barcode_idx_detected_at_post[bc_idx]

            if np.abs(bc_detected_at_pose[0] - detected_at_pose[0]) < 0.4 \
                and np.abs(bc_detected_at_pose[1] - detected_at_pose[1]) < 0.4 \
                and np.abs(bc_detected_at_pose[2] - detected_at_pose[2]) < 0.4:

                return bc_idx

    # If we get here, that means we have detected a barcode never seen before
    if list(barcode_idx_to_name.keys()):
        next_idx = max(list(barcode_idx_to_name.keys())) + 1
    else:
        next_idx = 0

    return next_idx


def get_sensor_reading(barcode_idx_to_name, barcode_idx_detected_at_post, x_est):

    barcode = get_bar_code()

    if not barcode:
        return None

    frame_center = (150, 200)
    frame_area = 150 * 200

    (x, y, w, h) = barcode.rect

    # get distance to bar code
    barcode_area = w * h

    distance = (barcode_area / frame_area) * 1.0  # might need to calibrate this number

    # Get bearing of bar code
    center = (x + w/2.0, y + h/2.0)

    angle = np.radians(10) if center[0] - frame_center[0] < 0 else np.radians(10)

    detected_at_pose = x_est[:3]

    barcode_name = barcode.data.decode()

    print(barcode_name)

    return distance, angle, get_barcode_idx(barcode_name,
                                            detected_at_pose,
                                            barcode_idx_to_name,
                                            barcode_idx_detected_at_post
                                            ), barcode_name


def get_pred_z(xv, barcode_idx, x_est):
    # predict the distance and bearing to a landmark given
    # current car pose
    # idx of the barcode

    # Get the estimated position of the barcode
    start_idx = 3 + barcode_idx * 2  # +3 because the first 3 vals are for the pose itself

    estimated_bc_pos = x_est[start_idx:start_idx+2]

    dx = estimated_bc_pos[0] - xv[0]
    dy = estimated_bc_pos[1] - xv[1]

    z = [0, 0]
    z[0] = np.sqrt(dx ** 2 + dy ** 2)
    z[1] = np.arctan2(dy, dx) - xv[2]

    return np.array(z)


def angdiff(th1, th2=None):
    "Modified from https://github.com/zenetio/Cabot/blob/9a6265b5a448561ffcf2810c3228d431630197a1/src/python/PurePursuit.py#L36"
    if th2 is not None:
        d = th1 - th2
        d = np.mod(d+np.pi, 2*np.pi) - np.pi
    else:
        d = np.mod(th1+np.pi, 2*np.pi) - np.pi

    return d


if __name__ == '__main__':

    conn_ok = connection_ok()
    print(f'Connection Ok: {conn_ok}')

    run_action('stop')

    # This is used to store detected bar code
    # Each key is the idx of the bar code
    # The value is of the form (bar code name, pose detected)
    barcode_idx_to_name = {}
    barcode_idx_detected_at_post = {}

    reset_car_state()

    x_est = np.array([0, 0, 0], dtype=np.float32)
    P_est = np.diag([.01, .01, 0.005]) ** 2

    hists = []

    num_iteration = int(80 * 3)

    for iteration in range(num_iteration):

        print(f'\nIteration: {iteration}\n')
        print(f'x_est: {x_est}')

        # sensor reading should contain distance, angle and barcode idx
        sensor_reading = get_sensor_reading(barcode_idx_to_name, barcode_idx_detected_at_post, x_est)

        print(f'sensor_reading: {sensor_reading}')

        drive_in_circle(20, angle=-5, run_time=0.5)

        odo = hard_code_odo()

        run_action('stop')

        # One step of the filter
        xv_est = x_est[:3]
        xm_est = x_est[3:]

        Pvv_est = P_est[:3, :3]

        if iteration > 0:
            Pvm_est = P_est[:3, 3:]
            Pmm_est = P_est[3:, 3:]

        else:
            Pvm_est = np.zeros((3, 1))
            Pmm_est = np.zeros((1, 1))

        xv_pred = get_next_pose_from_odo(xv_est, odo)

        Fx = get_Fx(xv_est, odo)
        # Pvv_pred = np.matmul(np.matmul(Fx, Pvv_est) Fx * Pvv_est * Fx.T

        # Pvv_pred = np.matmul(np.matmul(Fx, Pvv_est), Fx.T)

        Pvv_pred = Fx @ Pvv_est @ Fx.T

        # print('Fx', Fx)
        # print('Pvm_est', Pvm_est)

        Pvm_pred = np.matmul(Fx, Pvm_est)

        # print('Pvm_pred', Pvm_pred)
        # exit(0)
        Pmm_pred = Pmm_est
        xm_pred = xm_est

        # print('Pvv_pred', Pvv_pred)
        # print('Pvm_pred', Pvm_pred)

        # print(Pvm_pred.T)
        # print(Pmm_pred)

        # np.hstack((Pvm_pred.T, Pmm_pred))

        x_pred = np.concatenate((xv_pred, xm_pred))

        P_pred = np.vstack((
            np.hstack((Pvv_pred, Pvm_pred)),
            np.hstack((Pvm_pred.T, Pmm_pred))
        ))

        # Set these to None so that we can save them without complication later
        innov = None
        S = None
        K = None

        if sensor_reading is not None:
            z = sensor_reading[:2]
            bc_idx = sensor_reading[2]
            bc_name = sensor_reading[3]

            # if this is a barcode we never saw before
            if bc_idx not in barcode_idx_to_name:

                barcode_idx_to_name[bc_idx] = bc_name
                barcode_idx_detected_at_post[bc_idx] = xv_pred

                # Extend the state estimate
                distance = z[0]
                bearing = z[1] + xv_pred[2]  # bearing angle in vehicle frame

                bc_position = [xv_pred[0] + distance * np.cos(bearing), xv_pred[1] + distance * np.sin(bearing)]

                x_est = np.append(x_pred, bc_position)

                # Extend the covariance matrix
                # get the Jacobian for the new feature
                theta = xv_pred[2]
                r = z[0]
                bearing = z[1]
                Gz = np.array([
                    [np.cos(theta + bearing),   -r * np.sin(theta + bearing)],
                    [np.sin(theta + bearing),    r * np.cos(theta + bearing)]
                    ])

                # extend the covariance matrix            theta = xv(3);
                Gx = [[1,   0,   -r * np.sin(theta + bearing)],
                     [0,   1,    r * np.cos(theta + bearing)]]

                n = len(x_est) - 2 # -2 because n is supposed to be the length of x_est before appending new feature

                M_top = np.hstack((np.diag([1] * n), np.zeros((n, 2))))

                M_bot = np.hstack((Gx, np.zeros((2, n - 3)), Gz))

                M = np.vstack((M_top, M_bot))

                # P_ext = M * scipy.linalg.block_diag(P_est, np.zeros((2, 2))) * M.T\
                # print('M.shape', M.shape)
                # print('scipy.linalg.block_diag(P_est, np.zeros((2, 2)))', scipy.linalg.block_diag(P_est, np.zeros((2, 2))))

                P_est = M @ scipy.linalg.block_diag(P_est, np.zeros((2, 2))) @ M.T

            # If this is the bar code we saw before, perform the update step of the filter
            else:
                z_pred = get_pred_z(xv_pred, bc_idx, x_est)

                innov = np.array([0, 0])

                innov[0] = z[0] - z_pred[0]

                innov[1] = angdiff(z[1], z_pred[1])

                # xf is position of feature
                start_idx = 3 + bc_idx * 2  # +3 because the first 3 vals are for the pose itself

                xf = x_est[start_idx: start_idx+2]

                # Compute the jacobian for the feature (the k_th feature in this case)
                delta = xf - x_est[:2]

                r = scipy.linalg.norm(delta)

                Hx_k = [
                    [delta[0]/r,         delta[1]/r],
                    [0.0-delta[1]/(r ** 2),    delta[0]/(r ** 2)]
                ]

                # Compute the Jacobian for all features

                Hx = np.zeros((2, len(xm_pred)))

                # print(f'Hx_k: {Hx_k}')
                # print(f'Hx before: {Hx}')

                Hx[:, bc_idx*2:bc_idx*2+2] = Hx_k

                # print(f'Hx after: {Hx}')
                # print(f'delta: {delta}')

                Hxv = np.array([
                    [0.0 - delta[0]/r,    0.0 - delta[1]/r,        0],
                    [delta[1]/(r ** 2),  0.0-delta[0]/(r ** 2),   0.0 - 1]
                ])

                # print(f'Hxv: {Hxv} {Hxv.shape}')
                # print(f'Hx: {Hx} {Hx.shape}')

                Hx = np.hstack((Hxv, Hx))

                # Compute innovation covariance
                S = Hx @ P_pred @ Hx.T   # no W because we assume no noise

                # Compute Kalman gain
                K = P_pred @ Hx.T @ np.linalg.inv(S)

                # print(f'S: {S.shape}')
                # print(f'x_pred: {x_pred.shape}')
                # print(f'K: {K.shape}')
                # print(f'innov.T: {innov.T.shape}')

                # Update the state vector
                x_est = x_pred + K @ innov.T

                # Wrap heading state for the car
                x_est[2] = angdiff(x_est[2])

                # print(f'P_pred: {P_pred.shape}')

                # Update the covariance
                P_est = P_pred - K @ S @ K.T

                # Enforce P to be symmetric
                P_est = 0.5 * (P_est + P_est.T)

        hist = [x_est, odo, P_est, innov, S, K]

        hists.append(hist)

    with open(f'hist_{num_iteration}.pkl', 'wb') as f:
        pickle.dump(hists, f)
