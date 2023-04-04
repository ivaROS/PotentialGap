from __future__ import print_function
from future import standard_library
standard_library.install_aliases()
from builtins import str
from builtins import range
from builtins import object

import math
import numpy as np


from mpl_toolkits import mplot3d
from matplotlib import cm

import matplotlib.pyplot as plt
import yaml
import os
import rospkg

from matplotlib.ticker import FormatStrFormatter

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

		self.depl_a, self.depl_t, self.depl = self.creatLinearDecayEPL()
		self.depl_d = 0.2 * self.depl_t

		self.derl_a, self.derl_t, self.derl = self.creatLinearDecayERL()
		self.derl_d = 0.2 * self.derl_t

		# self.saveGeometry()

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

	def creatLinearDecayEPL(self):
		alpha = np.arccos(self.vec_dot_product).reshape((-1))
		alpha = np.flip(alpha)
		t = np.linspace(0, 6.5, 100)
		input_a, input_t = np.meshgrid(alpha, t)
		out_epl = np.zeros_like(input_a)
		
		for i in range(100):
			for j in range(self.num_samples):
				a = input_a[i,j]
				c_t = input_t[i,j]
				ang_rot = abs(a) - 0.5 * c_t
				if ang_rot < 0:
					c_ang = 0
				else:
					if a < 0:
						c_ang = -ang_rot
					else:
						c_ang = ang_rot
				
				cur_vec = np.array([math.cos(c_ang), math.sin(c_ang)])
				length_vec = self.robot.length / 2 * cur_vec
				if (c_ang <= -math.pi / 2) or (c_ang > 0 and c_ang <= math.pi / 2):
					left_pt_vec = np.array([-length_vec[1], length_vec[0]])
				else:
					left_pt_vec = np.array([length_vec[1], -length_vec[0]])

				left_pt_vec = left_pt_vec / np.linalg.norm(left_pt_vec)
				left_pt_vec = self.robot.width / 2 * left_pt_vec
				corner_pt = length_vec + left_pt_vec

				out_epl[i,j] = 2 * abs(corner_pt[1])
		
		return input_a, input_t, out_epl

	def creatLinearDecayERL(self):
		alpha = np.arccos(self.vec_dot_product).reshape((-1))
		# alpha = np.flip(alpha)
		t = np.linspace(0, 6.5, 50)
		input_a, input_t = np.meshgrid(alpha, t)
		out_er = np.zeros_like(input_a)
		
		for i in range(50):
			for j in range(self.num_samples):
				a = input_a[i,j]
				c_t = input_t[i,j]
				ang_rot = abs(a) - 0.5 * c_t
				if ang_rot < 0:
					c_ang = 0
				else:
					if a < 0:
						c_ang = -ang_rot
					else:
						c_ang = ang_rot
				
				robot_orient = np.array([1,0])
				gap_direct = np.array([math.cos(-c_ang), math.sin(-c_ang)])
				# rot_mat = np.array([[math.cos(c_ang), -math.sin(c_ang)],[math.sin(c_ang), math.cos(c_ang)]])
				# i_rot_mat = np.array([[math.cos(c_ang), math.sin(c_ang)],[-math.sin(c_ang), math.cos(c_ang)]])
				length_vec = self.robot.length / 2 * robot_orient
				d_vec = 0.2 * c_t * gap_direct
				step_size = 2 * math.pi / 100
				boud_dist = []
				for k in range(100):
					it_ang = k * step_size - math.pi
					cos_it_ang = math.cos(it_ang)
					if cos_it_ang > (self.robot.length / self.robot.diagonal_length) and cos_it_ang <= 1:
						dist = self.robot.length / 2. / cos_it_ang
					elif cos_it_ang <= (self.robot.length / self.robot.diagonal_length) and cos_it_ang > -(self.robot.length / self.robot.diagonal_length):
						dist = self.robot.width / 2. / (math.sqrt(1. - cos_it_ang ** 2))
					elif cos_it_ang <= -(self.robot.length / self.robot.diagonal_length) and cos_it_ang >= -1:
						dist = -self.robot.length / 2. / cos_it_ang

					it_vec = dist * np.array([math.cos(it_ang), math.sin(it_ang)])
					# rotated_vec = np.matmul(rot_mat, it_vec.reshape((-1,1)))
					b_vec = d_vec + it_vec
					boud_dist.append(np.linalg.norm(b_vec))

				p1 = d_vec + np.array([-self.robot.length / 2, 0])
				p2 = d_vec + np.array([-self.robot.length / 2, -self.robot.width / 2])
				p3 = d_vec + np.array([0, -self.robot.width / 2])
				p4 = d_vec + np.array([self.robot.length / 2, -self.robot.width / 2])
				p5 = d_vec + np.array([self.robot.length / 2, 0])
				p6 = d_vec + np.array([self.robot.length / 2, self.robot.width / 2])
				p7 = d_vec + np.array([0, self.robot.width / 2])
				p8 = d_vec + np.array([-self.robot.length / 2, self.robot.width / 2])

				boud_dist.append(np.linalg.norm(p1))
				boud_dist.append(np.linalg.norm(p2))
				boud_dist.append(np.linalg.norm(p3))
				boud_dist.append(np.linalg.norm(p4))
				boud_dist.append(np.linalg.norm(p5))
				boud_dist.append(np.linalg.norm(p6))
				boud_dist.append(np.linalg.norm(p7))
				boud_dist.append(np.linalg.norm(p8))

				min_dist = min(boud_dist)
				max_dist = max(boud_dist)
				
				# rot_d_vec = np.matmul(i_rot_mat, d_vec.reshape((-1,1)))
				# print(rot_d_vec.shape)

				if d_vec[0] > (-self.robot.length / 2) and d_vec[0] < (self.robot.length / 2) and d_vec[1] > (-self.robot.width / 2) and d_vec[1] < (self.robot.width / 2):
					out_er[i, j] = max_dist
					# out_er[i, j] = abs(max_dist - min_dist)
					# out_er[i,j] = 0
				else:
					out_er[i,j] = abs(max_dist - min_dist)

				# if a == 0:
				# 	print(out_er[i,j])
		return input_a, input_t, out_er

	def saveGeometry(self):
		result_dict = {'vec_dot_product' : self.vec_dot_product.reshape((-1)).tolist(),
					   'equivalent_radius' : self.robot_dist.reshape((-1)).tolist(),
					   'equivalent_pass_len' : self.equivalent_dist.reshape((-1)).tolist()}

		with open(self.yaml_file, 'w') as file:
			documents = yaml.dump(result_dict, file)

	def plotFigures(self):
		# plt.figure(1)
		# plt.plot(self.vec_dot_product, self.robot_dist)
		# plt.xlabel('(orient vector) dot (nearest pt vector)')
		# plt.ylabel('Equivalent Radius')

		# plt.figure(2)
		# plt.plot(self.vec_dot_product, self.equivalent_dist)
		# plt.xlabel('(orient vector) dot (motion vector)')
		# plt.ylabel('Equivalent passing length')

		# plt.show()

		fig = plt.figure(1)
		ax = plt.axes(projection='3d')
		# ax.plot_surface(self.depl_a, self.depl_t, self.depl,linewidth = 0, cmap=cm.coolwarm)
		ax.plot_surface(self.depl_a, self.depl_d, self.depl,linewidth = 0, cmap=cm.coolwarm)
		ax.invert_xaxis()
		ax.tick_params(axis='x', labelsize=40)
		ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
		ax.tick_params(axis='y', labelsize=40)
		ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
		ax.tick_params(axis='z', labelsize=40)
		ax.zaxis.set_major_formatter(FormatStrFormatter('%.1f'))
		start, end = ax.get_xlim()
		ax.xaxis.set_ticks(np.arange(start, end, (end - start)/4))
		start_y, end_y = ax.get_ylim()
		ax.yaxis.set_ticks(np.arange(start_y, end_y, (end_y - start_y)/4))
		start_z, end_z = ax.get_zlim()
		ax.zaxis.set_ticks(np.arange(start_z, end_z, (end_z - start_z)/4))
		fig.set_figwidth(15)
		fig.set_figheight(15)

		fig = plt.figure(2)
		ax1 = plt.axes(projection='3d')
		# ax1.plot_surface(self.derl_a, self.derl_t, self.derl,linewidth = 0, cmap=cm.coolwarm)
		ax1.plot_surface(self.derl_a, self.derl_d, self.derl,linewidth = 0, cmap=cm.coolwarm)
		ax1.invert_xaxis()
		ax1.tick_params(axis='x', labelsize=40)
		ax1.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
		ax1.tick_params(axis='y', labelsize=40)
		ax1.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))
		ax1.tick_params(axis='z', labelsize=40)
		ax1.zaxis.set_major_formatter(FormatStrFormatter('%.1f'))
		start, end = ax1.get_xlim()
		ax1.xaxis.set_ticks(np.arange(start, end, (end - start)/4))
		start_y, end_y = ax.get_ylim()
		ax1.yaxis.set_ticks(np.arange(start_y, end_y, (end_y - start_y)/4))
		start_z, end_z = ax.get_zlim()
		ax1.zaxis.set_ticks(np.arange(start_z, end_z, (end_z - start_z)/4))
		fig.set_figwidth(15)
		fig.set_figheight(15)


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
	file_name = shape + '_' + str(robot_num) + '_geometry_figure.yaml'
	yaml_file = config_path + file_name
	print(yaml_file)

	plot_figure = True
	pro = GeometryProcessor(robot, yaml_file, 0, num_samples=100, plot_figure=plot_figure)
