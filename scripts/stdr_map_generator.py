from nav_scripts.testing_scenarios import TestingScenarios
import rospy
import os
import subprocess
import time
import signal

# world_name = "campus"
world_name = ['fourth_floor'] #'campus'
# world_name = ['dense']
map_save_path = "/home/shiyu/potential_gap_ws/src/potential_gap/maps/"

launch_file_path = "/home/shiyu/potential_gap_ws/src/navigation_test/configs/launch/"

min_seed = 77
max_seed = 100

for wv, world in enumerate(world_name):
	if world is 'dense':
		map_save_subpath = world + '_0.75_maps/'
	else:
		map_save_subpath = world + '_maps/'

	if world is 'dense':
		launch_file_name = 'gazebo_empty_room_20x20_world.launch'
	else:
		launch_file_name = 'gazebo_' + world + '_world.launch'

	for seed in range(min_seed, max_seed):
		print 'Generate ' + world + ' map with seed ' + str(seed)
		# gazebo environment
		cmd_gazebo = str('roslaunch ' + launch_file_path + launch_file_name + ' gui:=false')

		subprocess.Popen(cmd_gazebo, shell=True)
		time.sleep(5)

		# spawn robots
		if world is 'dense':
			scenario_name = 'stereo_' + world
			task= {'scenario': scenario_name, 'seed': seed, 'min_obstacle_spacing': 0.75}
		else:
			scenario_name = 'stereo_' + world + '_obstacle'
			task= {'scenario': scenario_name, 'seed': seed, 'num_obstacles':50, 'min_obstacle_spacing': 1}

		
		rospy.init_node('test_driver', anonymous=True)
		scenarios = TestingScenarios()
		scenario = scenarios.getScenario(task)

		rospy.Rate(1).sleep()
		scenario.setupScenario()

		# open map saver
		# cmd_map_saver = str('rosrun map_server map_saver map:=/groundtruth/map'\
		# 					+ ' -f ' + map_save_path + map_save_subpath + str(seed))
		cmd_map_saver = str('cd ' + map_save_path + map_save_subpath + ' && ' \
							+ 'rosrun map_server map_saver map:=/groundtruth/map' \
							+ ' -f ' + str(seed))
		subprocess.Popen(cmd_map_saver, shell=True)

		# call map generator service
		cmd_call_service = str('rosservice call /gazebo_2dmap_plugin/generate_map')
		subprocess.Popen(cmd_call_service, shell=True)

		if world is 'campus':
			time.sleep(15)
		elif world is 'fourth_floor':
			time.sleep(15)
		else:
			time.sleep(20)

		# close
		subprocess.call('rosnode kill gazebo', shell=True)
		subprocess.call('rosnode kill map', shell=True)
