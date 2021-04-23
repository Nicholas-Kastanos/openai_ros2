#!/usr/bin/env python

import rclpy
from gazebo_msgs.msg import ODEPhysics
from gazebo_msgs.srv import SetPhysicsProperties
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from . import exceptions, service_utils


class GazeboConnection():

    def __init__(self, node, start_init_physics_parameters, reset_world_or_sim, max_retry = 20):
        self.node = node

        self._max_retry = max_retry

        self.unpause_client = service_utils.create_service_client(self.node, Empty, '/unpause_physics')
        self.pause_client = service_utils.create_service_client(self.node, Empty, '/pause_physics')
        self.reset_simulation_client = service_utils.create_service_client(self.node, Empty, '/reset_simulation')
        self.reset_world_client = service_utils.create_service_client(self.node, Empty, '/reset_world')

        ### This code still needs to be migrated to ROS2
        # # Setup the Gravity Controle system
        # service_name = '/gazebo/set_physics_properties'
        # rospy.logdebug("Waiting for service " + str(service_name))
        # rospy.wait_for_service(service_name)
        # rospy.logdebug("Service Found " + str(service_name))
        # self.set_physics = rospy.ServiceProxy(service_name, SetPhysicsProperties)
        # self.set_physics_client = service_utils.create_service_client(self.node, SetPhysicsProperties, '/gazebo/set_physics_properties')

        self.start_init_physics_parameters = start_init_physics_parameters
        self.reset_world_or_sim = reset_world_or_sim
        self.init_values()
        # We always pause the simulation, important for legged robots learning
        self.pauseSim()

    def pauseSim(self):
        self.node.get_logger().debug("PAUSING service found...")
        paused_done = False
        counter = 0
        
        while not paused_done:
            if counter < self._max_retry:
                try:
                    self.node.get_logger().debug("PAUSING service calling...")
                    service_utils.call_and_wait_for_service_response(self.node, self.pause_client, Empty.Request())
                    paused_done = True
                    self.node.get_logger().debug("PAUSING service calling...DONE")
                except Exception as e:
                    counter += 1
                    self.node.get_logger().error("/pause_physics service call failed. Retrying "+str(counter) + str(e))
            else:
                error_message = "Maximum retries done ("+str(self._max_retry)+"), please check Gazebo pause service"
                self.node.get_logger().fatal(error_message)
                assert False, error_message

        self.node.get_logger().debug("PAUSING FINISH")

    def unpauseSim(self):
        self.node.get_logger().debug("UNPAUSING service found...")
        unpaused_done = False
        counter = 0
        while not unpaused_done:
            if counter < self._max_retry:
                try:
                    self.node.get_logger().debug("UNPAUSING service calling...")
                    service_utils.call_and_wait_for_service_response(self.node, self.unpause_client, Empty.Request())
                    unpaused_done = True
                    self.node.get_logger().debug("UNPAUSING service calling...DONE")
                except Exception as e:
                    counter += 1
                    self.node.get_logger().error("/unpause_physics service call failed...Retrying "+str(counter))
            else:
                error_message = "Maximum retries done"+str(self._max_retry)+", please check Gazebo unpause service"
                self.node.get_logger().fatal(error_message)
                assert False, error_message

        self.node.get_logger().debug("UNPAUSING FINISH")


    def resetSim(self):
        """
        This was implemented because some simulations, when reseted the simulation
        the systems that work with TF break, and because sometime we wont be able to change them
        we need to reset world that ONLY resets the object position, not the entire simulation
        systems.
        """
        if self.reset_world_or_sim == "SIMULATION":
            self.node.get_logger().error("SIMULATION RESET")
            self.resetSimulation()
        elif self.reset_world_or_sim == "WORLD":
            self.node.get_logger().error("WORLD RESET")
            self.resetWorld()
        elif self.reset_world_or_sim == "NO_RESET_SIM":
            self.node.get_logger().error("NO RESET SIMULATION SELECTED")
        else:
            self.node.get_logger().error("WRONG Reset Option:"+str(self.reset_world_or_sim))

    def resetSimulation(self):
        try:
            service_utils.call_and_wait_for_service_response(self.node, self.reset_simulation_client, Empty.Request())
        except Exception as e:
            self.node.get_logger().error("/reset_simulation service call failed")

    def resetWorld(self):
        try:
            service_utils.call_and_wait_for_service_response(self.node, self.reset_world_client, Empty.Request())
        except Exception as e:
            self.node.get_logger().error("/reset_world service call failed")

    def init_values(self):

        self.resetSim()

        if self.start_init_physics_parameters:
            self.node.get_logger().debug("Initialising Simulation Physics Parameters")
            self.init_physics_parameters()
        else:
            self.node.get_logger().error("NOT Initialising Simulation Physics Parameters")

    def init_physics_parameters(self):
        raise NotImplementedError
    ### This code needs to be migrated to ROS2

    #     """
    #     We initialise the physics parameters of the simulation, like gravity,
    #     friction coeficients and so on.
    #     """
    #     self._time_step = Float64(0.001) # pylint: disable=too-many-function-args
    #     self._max_update_rate = Float64(1000.0) # pylint: disable=too-many-function-args

    #     self._gravity = Vector3()
    #     self._gravity.x = 0.0
    #     self._gravity.y = 0.0
    #     self._gravity.z = -9.81

    #     self._ode_config = ODEPhysics()
    #     self._ode_config.auto_disable_bodies = False
    #     self._ode_config.sor_pgs_precon_iters = 0
    #     self._ode_config.sor_pgs_iters = 50
    #     self._ode_config.sor_pgs_w = 1.3
    #     self._ode_config.sor_pgs_rms_error_tol = 0.0
    #     self._ode_config.contact_surface_layer = 0.001
    #     self._ode_config.contact_max_correcting_vel = 0.0
    #     self._ode_config.cfm = 0.0
    #     self._ode_config.erp = 0.2
    #     self._ode_config.max_contacts = 20

    #     self.update_gravity_call()


    # def update_gravity_call(self):

    #     self.pauseSim()

    #     set_physics_request = SetPhysicsProperties.Request()
    #     set_physics_request.time_step = self._time_step.data
    #     set_physics_request.max_update_rate = self._max_update_rate.data
    #     set_physics_request.gravity = self._gravity
    #     set_physics_request.ode_config = self._ode_config

    #     # rospy.logdebug(str(set_physics_request.gravity))
    #     self.node.get_logger().debug(str(set_physics_request.gravity))

    #     # result = self.set_physics(set_physics_request)
    #     result = service_utils.call_and_wait_for_service_response(self.node, self.set_physics_client, set_physics_request)

    #     # rospy.logdebug("Gravity Update Result==" + str(result.success) + ",message==" + str(result.status_message))
    #     self.node.get_logger().debug("Gravity Update Result==" + str(result.success) + ",message==" + str(result.status_message))

    #     self.unpauseSim()

    # def change_gravity(self, x, y, z):
    #     self._gravity.x = x
    #     self._gravity.y = y
    #     self._gravity.z = z

    #     self.update_gravity_call()
