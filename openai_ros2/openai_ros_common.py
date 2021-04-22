#!/usr/bin/env python
import gym # pylint: disable=import-error
from .task_envs.task_envs_list import RegisterOpenAI_Ros_Env
import roslaunch # pylint: disable=import-error
import rclpy
from rclpy.node import Node
import rospkg
import os
import git # pylint: disable=import-error
import sys
import subprocess


def StartOpenAI_ROS_Environment(node: Node, task_and_robot_environment_name):
    """
    It Does all the stuff that the user would have to do to make it simpler
    for the user.
    This means:
    0) Registers the TaskEnvironment wanted, if it exists in the Task_Envs.
    2) Checks that the workspace of the user has all that is needed for launching this.
    Which means that it will check that the robot spawn launch is there and the worls spawn is there.
    4) Launches the world launch and the robot spawn.
    5) It will import the Gym Env and Make it.
    """
    node.get_logger().warning("Env: {} will be imported".format(
        task_and_robot_environment_name))
    result = RegisterOpenAI_Ros_Env(task_env=task_and_robot_environment_name,
                                    max_episode_steps=10000)

    if result:
        node.get_logger().warning("Register of Task Env went OK, lets make the env..."+str(task_and_robot_environment_name))
        env = gym.make(task_and_robot_environment_name)
    else:
        node.get_logger().warning("Something Went wrong in the register")
        env = None

    return env


class ROSLauncher(object):
    def __init__(self, node: Node, rospackage_name, launch_file_name, ros_ws_abspath="/home/user/simulation_ws"):

        self._rospackage_name = rospackage_name
        self._launch_file_name = launch_file_name

        self.rospack = rospkg.RosPack()

        # Check Package Exists
        try:
            pkg_path = self.rospack.get_path(rospackage_name)
            node.get_logger().debug("Package FOUND...")
        except rospkg.common.ResourceNotFound:
            node.get_logger().warning("Package NOT FOUND, lets Download it...")
            pkg_path = self.DownloadRepo(node, package_name=rospackage_name,
                                         ros_ws_abspath=ros_ws_abspath)

        # Now we check that the Package path is inside the ros_ws_abspath
        # This is to force the system to have the packages in that ws, and not in another.
        if ros_ws_abspath in pkg_path:
            node.get_logger().debug("Package FOUND in the correct WS!")
        else:
            node.get_logger().warning("Package FOUND in "+pkg_path +
                          ", BUT not in the ws="+ros_ws_abspath+", lets Download it...")
            pkg_path = self.DownloadRepo(node, package_name=rospackage_name,
                                         ros_ws_abspath=ros_ws_abspath)

        # If the package was found then we launch
        if pkg_path:
            node.get_logger().info(
                ">>>>>>>>>>Package found in workspace-->"+str(pkg_path))
            launch_dir = os.path.join(pkg_path, "launch")
            path_launch_file_name = os.path.join(launch_dir, launch_file_name)

            node.get_logger().warning("path_launch_file_name=="+str(path_launch_file_name))

            source_env_command = "source "+ros_ws_abspath+"/devel/setup.bash;"
            roslaunch_command = "roslaunch  {0} {1}".format(rospackage_name, launch_file_name)
            command = source_env_command+roslaunch_command
            node.get_logger().warning("Launching command="+str(command))

            p = subprocess.Popen(command, shell=True)

            state = p.poll()
            if state is None:
                node.get_logger().info("process is running fine")
            elif state < 0:
                node.get_logger().info("Process terminated with error")
            elif state > 0:
                node.get_logger().info("Process terminated without error")
            """
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(self.uuid)
            self.launch = roslaunch.parent.ROSLaunchParent(
                self.uuid, [path_launch_file_name])
            self.launch.start()
            """


            node.get_logger().info(">>>>>>>>>STARTED Roslaunch-->" +
                          str(self._launch_file_name))
        else:
            assert False, "No Package Path was found for ROS apckage ==>" + \
                str(rospackage_name)

    def DownloadRepo(self, node: Node, package_name, ros_ws_abspath):
        """
        This has to be installed
        sudo pip install gitpython
        """
        commands_to_take_effect = "\nIn a new Shell:::>\ncd "+ros_ws_abspath + \
            "\ncatkin_make\nsource devel/setup.bash\nrospack profile\n"
        commands_to_take_effect2 = "\nIn your deeplearning program execute shell catkin_ws:::>\ncd /home/user/catkin_ws\nsource devel/setup.bash\nrospack profile\n"

        ros_ws_src_abspath_src = os.path.join(ros_ws_abspath, "src")
        pkg_path = None
        # We retrieve the got for the package asked
        package_git = None
        package_to_branch_dict = {}

        node.get_logger().debug("package_name===>"+str(package_name)+"<===")

        if package_name == "moving_cube_description":

            url_git_1 = "https://bitbucket.org/theconstructcore/moving_cube.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "master"

            url_git_2 = "https://bitbucket.org/theconstructcore/spawn_robot_tools.git"
            package_git.append(url_git_2)
            package_to_branch_dict[url_git_2] = "master"

        elif package_name == "rosbot_gazebo" or package_name == "rosbot_description":
            package_git = [
                "https://bitbucket.org/theconstructcore/rosbot_husarion.git"]
            package_git.append(
                "https://github.com/paulbovbel/frontier_exploration.git")

        elif package_name == "fetch_gazebo":
            package_git = [
                "https://bitbucket.org/theconstructcore/fetch_tc.git"]

        elif package_name == "cartpole_description" or package_name == "cartpole_v0_training":
            package_git = [
                "https://bitbucket.org/theconstructcore/cart_pole.git"]

        elif package_name == "legged_robots_sims" or package_name == "legged_robots_description" or package_name == "my_legged_robots_description" or package_name == "my_legged_robots_sims" or package_name == "my_hopper_training":
            package_git = ["https://bitbucket.org/theconstructcore/hopper.git"]

        elif package_name == "iri_wam_description" or package_name == "iri_wam_gazebo" or package_name == "iri_wam_reproduce_trajectory" or package_name == "iri_wam_aff_demo":
            url_git_1 = "https://bitbucket.org/theconstructcore/iri_wam.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "noetic"
            
            url_git_2 = "https://bitbucket.org/theconstructcore/hokuyo_model.git"
            package_git.append(url_git_2)
            package_to_branch_dict[url_git_2] = "master"


        elif package_name == "drone_construct" or package_name == "drone_demo" or package_name == "sjtu_drone" or package_name == "custom_teleop" or package_name == "ardrone_as":
            url_git_1 = "https://bitbucket.org/theconstructcore/parrot_ardrone.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "kinetic-gazebo9"

        elif package_name == "sawyer_gazebo":
            url_git_1 = "https://bitbucket.org/theconstructcore/sawyer_full.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "update2019"

        elif package_name == "shadow_gazebo":

            url_git_1 = "https://bitbucket.org/theconstructcore/shadow_robot_smart_grasping_sandbox.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "melodic-gazebo9"


            url_git_2 = "https://github.com/ros-industrial/universal_robot.git"
            package_git.append(url_git_2)
            package_to_branch_dict[url_git_2] = "melodic-devel"

        elif package_name == "summit_xl_gazebo":

            url_git_1 = "https://bitbucket.org/theconstructcore/summit_xl.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "melodic-gazebo9"

            url_git_2 = "https://github.com/RobotnikAutomation/robotnik_msgs.git"
            package_git.append(url_git_2)
            package_to_branch_dict[url_git_2] = "master"

            url_git_3 = "https://github.com/tu-darmstadt-ros-pkg/hector_gazebo.git"
            package_git.append(url_git_3)
            package_to_branch_dict[url_git_3] = "melodic-devel"

        elif package_name == "gym_construct":
            package_git = [
                "https://bitbucket.org/theconstructcore/open_ai_gym_construct.git"]

        elif package_name == "turtlebot_gazebo":

            url_git_1 = "https://bitbucket.org/theconstructcore/turtlebot.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "kinetic-gazebo9"


        elif package_name == "turtlebot3_gazebo":

            url_git_1 = "https://bitbucket.org/theconstructcore/turtlebot3.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "master"

        elif package_name == "robotx_gazebo":
            url_git_1 = "https://bitbucket.org/theconstructcore/vmrc.git"
            package_git = [url_git_1]
            package_to_branch_dict[url_git_1] = "master"

            url_git_2 = "https://bitbucket.org/theconstructcore/spawn_robot_tools.git"
            package_git.append(url_git_2)
            package_to_branch_dict[url_git_2] = "master"

        elif package_name == "fetch_simple_description":
            package_git = [
                "https://bitbucket.org/theconstructcore/fetch_simple_simulation.git"]
            package_git.append(
                "https://bitbucket.org/theconstructcore/spawn_robot_tools.git")

        # ADD HERE THE GITs List To Your Simuation

        else:
            node.get_logger().error("Package [ >"+package_name +
                         "< ] is not supported for autodownload, do it manually into >"+str(ros_ws_abspath))
            assert False, "The package " + \
                " is not supported, please check the package name and the git support in openai_ros_common.py"

        # If a Git for the package is supported
        if package_git:
            for git_url in package_git:
                try:
                    node.get_logger().debug("Lets download git="+git_url +
                                   ", in ws="+ros_ws_src_abspath_src)
                    if git_url in package_to_branch_dict:
                        branch_repo_name = package_to_branch_dict[git_url]
                        git.Git(ros_ws_src_abspath_src).clone(git_url,branch=branch_repo_name)
                    else:
                        git.Git(ros_ws_src_abspath_src).clone(git_url)

                    node.get_logger().debug("Download git="+git_url +
                                   ", in ws="+ros_ws_src_abspath_src+"...DONE")
                except git.exc.GitCommandError as e:
                    node.get_logger().warning(str(e))
                    node.get_logger().warning("The Git "+git_url+" already exists in " +
                                  ros_ws_src_abspath_src+", not downloading")

            # We check that the package is there
            try:
                pkg_path = self.rospack.get_path(package_name)
                node.get_logger().warning("The package "+package_name+" was FOUND by ROS.")

                if ros_ws_abspath in pkg_path:
                    node.get_logger().debug("Package FOUND in the correct WS!")
                else:
                    node.get_logger().warning("Package FOUND in="+pkg_path +
                                  ", BUT not in the ws="+ros_ws_abspath)
                    node.get_logger().error(
                        "IMPORTANT!: You need to execute the following commands and rerun to dowloads to take effect.")
                    node.get_logger().error(commands_to_take_effect)
                    node.get_logger().error(commands_to_take_effect2)
                    sys.exit()

            except rospkg.common.ResourceNotFound:
                node.get_logger().error("Package "+package_name+" NOT FOUND by ROS.")
                # We have to make the user compile and source to make ROS be able to find the new packages
                # TODO: Make this automatic
                node.get_logger().error(
                    "IMPORTANT!: You need to execute the following commands and rerun to dowloads to take effect.")
                node.get_logger().error(commands_to_take_effect)
                node.get_logger().error(commands_to_take_effect2)
                sys.exit()

        return pkg_path
