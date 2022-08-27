from __future__ import print_function
from future import standard_library
standard_library.install_aliases()
from builtins import str
from builtins import range
from builtins import object

import math
import numpy as np

import matplotlib.pyplot as plt
import yaml
import os
import rospkg

class Robot(object):
	shape = None
	radius = None
	length = None
	width = None
	diagonal_length = None

	def __init__(self, shape, radius=None, length=None, width=None):
		self.shape = shape
		if self.shape == "circle":
			if radius is not None:
				self.radius = radius
			else:
				raise RuntimeError("Circle robot radius is not given.")
		elif self.shape == "box":
			if length is not None and width is not None:
				self.length = length
				self.width = width
				self.diagonal_length = math.sqrt(length ** 2 + width ** 2)
			else:
				raise RuntimeError("Box robot geometry is not given.")
		else:
			raise RuntimeError("The given robot shape is not supported.")

class GeometryProcessor(object):
	robot = None
	yaml_file = None

	orient_min = None
	orient_max = None
	num_samples = None

	yaw_list = None

	def __init__(self, robot, yaml_file, orient_min=-math.pi, orient_max=math.pi, num_samples=1000, plot_figure=False):
		self.robot = robot
		self.yaml_file = yaml_file

		self.orient_min = orient_min
		self.orient_max = orient_max
		self.num_samples = num_samples

		self.generateSamples()

		self.generateRobotDist()
		self.generateEquivalentDist()

		self.saveGeometry()

		if plot_figure:
			self.plotFigures()


	def generateSamples(self):
		self.yaw_list = np.linspace(self.orient_min, self.orient_max, self.num_samples).reshape((-1,1))
		self.orientation_vectors = np.hstack((np.cos(self.yaw_list), np.sin(self.yaw_list)))
		
		self.motion_list = self.yaw_list
		self.motion_vectors = self.orientation_vectors

		self.static_motion_vector = np.array([1., 0]).reshape((-1,1))

		self.vec_dot_product = np.matmul(self.orientation_vectors, self.static_motion_vector)
		self.vec_dot_product = np.flip(self.vec_dot_product)

	def generateRobotDist(self):
		self.robot_dist = np.zeros_like(self.vec_dot_product)

		for i in range(self.num_samples):
			if self.robot.shape == 'box':
				if self.vec_dot_product[i] > (self.robot.length / self.robot.diagonal_length) and self.vec_dot_product[i] <= 1:
					self.robot_dist[i] = self.robot.length / 2. / self.vec_dot_product[i]
				elif self.vec_dot_product[i] <= (self.robot.length / self.robot.diagonal_length) and self.vec_dot_product[i] > -(self.robot.length / self.robot.diagonal_length):
					self.robot_dist[i] = self.robot.width / 2. / (math.sqrt(1. - self.vec_dot_product[i] ** 2))
				elif self.vec_dot_product[i] <= -(self.robot.length / self.robot.diagonal_length) and self.vec_dot_product[i] >= -1:
					self.robot_dist[i] = -self.robot.length / 2. / self.vec_dot_product[i]
			elif self.robot.shape == 'circle':
				self.robot_dist[i] = self.robot.radius

	def generateEquivalentDist(self):
		self.equivalent_dist = np.zeros_like(self.vec_dot_product)

		for i in range(self.num_samples):
			if self.robot.shape == 'box':
				length_vec = self.robot.length / 2 * self.orientation_vectors[i, :]

				left_pt_vec = np.array([-length_vec[1], length_vec[0]])
				if self.vec_dot_product[i] < 0:
					# if (length_vec[0]*left_pt_vec[1] - length_vec[1]*left_pt_vec[0]) < 0:
					left_pt_vec = -left_pt_vec
				left_pt_vec = left_pt_vec / np.linalg.norm(left_pt_vec)
				left_pt_vec = self.robot.width / 2 * left_pt_vec
				corner_pt = length_vec + left_pt_vec

				self.equivalent_dist[i] = 2 * abs(corner_pt[1])
			elif self.robot.shape == 'circle':
				self.equivalent_dist[i] = 2 * self.robot.radius

	def saveGeometry(self):
		result_dict = {'vec_dot_product' : self.vec_dot_product.reshape((-1)).tolist(),
					   'equivalent_radius' : self.robot_dist.reshape((-1)).tolist(),
					   'equivalent_pass_len' : self.equivalent_dist.reshape((-1)).tolist()}

		with open(self.yaml_file, 'w') as file:
			documents = yaml.dump(result_dict, file)

	def plotFigures(self):
		plt.figure(1)
		plt.plot(self.vec_dot_product, self.robot_dist)
		plt.xlabel('(orient vector) dot (nearest pt vector)')
		plt.ylabel('Equivalent Radius')

		plt.figure(2)
		plt.plot(self.vec_dot_product, self.equivalent_dist)
		plt.xlabel('(orient vector) dot (motion vector)')
		plt.ylabel('Equivalent passing length')

		plt.show()


if __name__ == '__main__':
	shape = 'box'

	if shape == 'box':
		robot = Robot("box", length=0.7, width=0.3)
	elif shape == 'circle':
		robot = Robot("circle", radius=0.2)

	rospack = rospkg.RosPack()
	config_path = rospack.get_path('quadruped_nav_benchmark') + '/config/robot_geometry/'
	if not os.path.exists(config_path):
		raise RuntimeError('Saved path doesn not exist.')

	robot_num = 1
	file_name = shape + '_' + str(robot_num) + '_geometry.yaml'
	yaml_file = config_path + file_name
	print(yaml_file)

	plot_figure = False
	pro = GeometryProcessor(robot, yaml_file, 0, num_samples=1000, plot_figure=plot_figure)