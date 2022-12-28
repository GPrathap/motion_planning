#!usr/bin/env python
import rospy
import numpy as np
import math
import cvxpy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from prius_msgs.msg import Control
from geometry_msgs.msg import PoseStamped

global current_state
global path_message
global state_ref
global d_ref

max_index = 0
target_vel = 10
NX = 4
NU = 2
horizon_length = 20
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 0.01])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 1.0, 1.0])  # state cost matrix
Qf = Q  # state final matrix
WB = 1.983  # [m]
MAX_ITER = 3
DU_TH = 0.1
DT = 0.2
MAX_STEER = math.radians(30.0)  # maximum steering angle [rad]
MAX_DSTEER = math.radians(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 15.0  # maximum speed [m/s]
MIN_SPEED = -10.0  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]
oa, odelta = None, None

class State:

	def __init__(self, x = 0, y = 0, v = 0, yaw = 0):
		self.x = x
		self.y = y
		self.v = v
		self.yaw = yaw
		self.predelta = None

	def update(self, data):
		print "updating state \n"
		self.x = data.pose.pose.position.x
		self.y = data.pose.pose.position.y

		# quarternion to euler conversion
		siny = +2.0 * (data.pose.pose.orientation.w *
					   data.pose.pose.orientation.z +
					   data.pose.pose.orientation.x *
					   data.pose.pose.orientation.y)
		cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
							 data.pose.pose.orientation.y +
							 data.pose.pose.orientation.z *
							 data.pose.pose.orientation.z)
		self.yaw = math.atan2(siny, cosy) # yaw in radians  
		
		self.v = (data.twist.twist.linear.x * math.cos(self.yaw) +
					data.twist.twist.linear.y * math.sin(self.yaw))

		print self.x, self.y, self.v, self.yaw, "\n"

current_state = State(x=-0.0, y=-0.0, yaw=0.0, v=0.0)

def dist(a,x,y):
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5


def prius_pub(data):
	'''
	publishes the velocity and steering angle
	published on topic : ackermann_cmd_topic
	'''
	global prius_vel
	prius_vel = Control()

	if(data.linear.x > 0):
		prius_vel.throttle = data.linear.x 
		prius_vel.brake = 0
		# print ("acc")
		# print (prius_vel.throttle)

	if(data.linear.x < 0):
		prius_vel.brake = -data.linear.x 
		prius_vel.throttle = 0
		# print ("brake")
		# print (prius_vel.brake)

	prius_vel.steer = data.angular.z / 30
	#print "steering:", prius_vel.steer

	pub_vel.publish(prius_vel)

def calc_nearest_index(state, path_msg):
	'''
	calculates minimum distance between path and a given point
	'''
	distances = []

	for i in range(len(path_msg.poses)):
		a = path_msg.poses[i]
		distances += [dist(a, state.x, state.y)]   
	ep = min(distances) 
	cp = distances.index(ep)
	return cp

def calc_ref_trajectory(path_msg):
	ref_state = np.zeros((NX, horizon_length + 1))
	ref_d = np.zeros((1, horizon_length + 1))
	global current_state
	nearest_index = calc_nearest_index(current_state, path_msg)
	print "nearest index is: ", nearest_index, "\n"
	for i in range(horizon_length + 1):
		if nearest_index + i < max_index:
			siny_path = +2.0 * (path_msg.poses[nearest_index + i].pose.orientation.w *
						   path_msg.poses[nearest_index + i].pose.orientation.z +
						   path_msg.poses[nearest_index + i].pose.orientation.x *
						   path_msg.poses[nearest_index + i].pose.orientation.y)
			cosy_path = +1.0 - 2.0 * (path_msg.poses[nearest_index + i].pose.orientation.y *
								 path_msg.poses[nearest_index + i].pose.orientation.y +
								 path_msg.poses[nearest_index + i].pose.orientation.z *
								 path_msg.poses[nearest_index + i].pose.orientation.z)
			path_yaw = math.atan2(siny_path, cosy_path) # yaw in radians
			ref_state[0,i] = path_msg.poses[nearest_index + i].pose.position.x
			ref_state[1,i] = path_msg.poses[nearest_index + i].pose.position.y
			ref_state[2,i] = target_vel
			ref_state[3,i] = path_yaw
			ref_d[0,i] = 0.0
		else:
			siny_path = +2.0 * (path_msg.poses[max_index].pose.orientation.w *
						   path_msg.poses[max_index].pose.orientation.z +
						   path_msg.poses[max_index].pose.orientation.x *
						   path_msg.poses[max_index].pose.orientation.y)
			cosy_path = +1.0 - 2.0 * (path_msg.poses[max_index].pose.orientation.y *
								 path_msg.poses[max_index].pose.orientation.y +
								 path_msg.poses[max_index].pose.orientation.z *
								 path_msg.poses[max_index].pose.orientation.z)
			path_yaw = math.atan2(siny_path, cosy_path) # yaw in radians
			ref_state[0,i] = path_msg.poses[max_index].pose.position.x
			ref_state[1,i] = path_msg.poses[max_index].pose.position.y
			ref_state[2,i] = target_vel
			ref_state[3,i] = path_yaw
			ref_d[0,i] = 0.0
	
	return ref_state, ref_d

def get_nparray_from_matrix(x):
	return np.array(x).flatten()

def predict_motion(x0, oa, od, xref):
	xbar = xref * 0.0
	for i in range(len(x0)):
		xbar[i, 0] = x0[i]

	state = State(x=x0[0], y=x0[1], yaw=x0[3], v=x0[2])
	for (ai, di, i) in zip(oa, od, range(1, horizon_length + 1)):
		state = update_state(state, ai, di)
		xbar[0, i] = state.x
		xbar[1, i] = state.y
		xbar[2, i] = state.v
		xbar[3, i] = state.yaw

	return xbar

def update_state(state, a, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * DT
    state.y = state.y + state.v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * DT
    state.v = state.v + a * DT

    if state. v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state. v < MIN_SPEED:
        state.v = MIN_SPEED

    return state

def get_linear_model_matrix(v, phi, delta):

    A = np.matrix(np.zeros((NX, NX)))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.matrix(np.zeros((NX, NU)))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C

def iterative_linear_mpc_control(xref, x0, dref, oa, od):
	"""
	MPC contorl with updating operational point iteraitvely
	"""

	if oa is None or od is None:
		oa = [0.0] * horizon_length
		od = [0.0] * horizon_length

	for i in range(MAX_ITER):
		print "optimal acceleration is: ", oa, "\n"
		xbar = predict_motion(x0, oa, od, xref)
		poa, pod = oa[:], od[:]
		oa, od, ox, oy, oyaw, ov = linear_mpc_control(xref, xbar, x0, dref)
		du = sum(abs(oa - poa)) + sum(abs(od - pod))  # calc u change value
		if du <= DU_TH:
			break
	else:
		print "Iterative is max iter" 

	return oa, od, ox, oy, oyaw, ov


def linear_mpc_control(xref, xbar, x0, dref):
	"""
	linear mpc control
	xref: reference point
	xbar: operational point
	x0: initial state
	dref: reference steer angle
	"""

	x = cvxpy.Variable((NX, horizon_length + 1))
	u = cvxpy.Variable((NU, horizon_length))

	cost = 0.0
	constraints = []

	for t in range(horizon_length):
		cost += cvxpy.quad_form(u[:, t], R)

		if t != 0:
			cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

		A, B, C = get_linear_model_matrix(
			xbar[2, t], xbar[3, t], dref[0, t])
		constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

		if t < (horizon_length - 1):
			cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
			constraints += [cvxpy.abs(u[1, t + 1] - u[1, t])
							<= MAX_DSTEER * DT]

	cost += cvxpy.quad_form(xref[:, horizon_length] - x[:, horizon_length], Qf)

	constraints += [x[:, 0] == x0]
	constraints += [x[2, :] <= MAX_SPEED]
	constraints += [x[2, :] >= MIN_SPEED]
	constraints += [cvxpy.abs(u[0, :]) <= MAX_ACCEL]
	constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]

	prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
	prob.solve(solver=cvxpy.ECOS, verbose=False)

	if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
		ox = get_nparray_from_matrix(x.value[0, :])
		oy = get_nparray_from_matrix(x.value[1, :])
		ov = get_nparray_from_matrix(x.value[2, :])
		oyaw = get_nparray_from_matrix(x.value[3, :])
		oa = get_nparray_from_matrix(u.value[0, :])
		odelta = get_nparray_from_matrix(u.value[1, :])

	else:
		print "Error: Cannot solve mpc.." 
		oa, odelta, ox, oy, oyaw, ov = None, None, None, None, None, None

	return oa, odelta, ox, oy, oyaw, ov

def callback_path(data):
	global path_message
	global max_index
	path_message = data
	max_index = len(data.poses)


def callback_vel(data):
	global target_vel
	target_vel = data.linear.x

def callback_feedback(data):
	global state_ref
	global target_vel
	global current_state
	global d_ref
	global path_message
	global oa
	global odelta
	global r

	output = Twist()
	optimal_path = Path()
	optimal_path.header.frame_id = rospy.get_param('~output_frame', 'map')
	print "entered odom_callback \n"
	print "current_state is ", data.pose.pose.position.x, data.pose.pose.position.y, data.twist.twist.linear.x
	current_state.update(data)
	# print current_state.x, current_state.y, current_state.v, current_state.yaw, "\n"
	state_ref, d_ref = calc_ref_trajectory(path_message)
	# print "reference state is: ", state_ref, "\n"

	x0 = [current_state.x, current_state.y, current_state.v, current_state.yaw]

	oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(state_ref, x0, d_ref, oa, odelta)
	print "optimal x is: ", ox, "\n"
	print "optimal y is: ", oy, "\n"
	print "optimal yaw is: ", oyaw, "\n"
	print "optimal v is: ", ov, "\n"

	for i in range(len(ox)):
		pose = PoseStamped()
		pose.pose.position.x = ox[i]
		pose.pose.position.y = oy[i]
		q = tf.transformations.quaternion_from_euler(0, 0, oyaw[i])
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]

		optimal_path.poses.append(pose)

	if odelta is not None:
		di, ai = odelta[0], oa[0]

	print "acceleration: ", oa, "\n"
	print "steering: ", odelta, "\n"
	print "\n"

	output.linear.x = ai
	output.angular.z = di * 180 / math.pi

	prius_pub(output)
	pub_ref.publish(optimal_path)


def start_mpc():
	global pub_vel
	global pub_ref
	
	ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/prius')
	
	rospy.init_node("model_predictive_control", anonymous=True)
	
	pub_vel = rospy.Publisher(ackermann_cmd_topic, Control, queue_size=10)
	pub_ref = rospy.Publisher("reference_path", Path, queue_size=10)
	
	rospy.Subscriber("cmd_vel", Twist, callback_vel, queue_size=1)
	rospy.Subscriber("astroid_path", Path, callback_path, queue_size=1)
	rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback, queue_size=1)

	rospy.spin()

if __name__ == '__main__':
	start_mpc()