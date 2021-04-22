#!/usr/bin/env python

import rclpy
from controller_manager_msgs.srv import SwitchController

from . import exceptions, service_utils


class ControllersConnection():
    
    def __init__(self, node, namespace, controllers_list):
        self.node = node

        self.node.get_logger().warning("Start Init ControllersConnection")
        self.controllers_list = controllers_list
  
        self.switch_service_name = '/'+namespace+'/controller_manager/switch_controller'
        # self.switch_service = rospy.ServiceProxy(self.switch_service_name, SwitchController)
        self.switch_service_client = service_utils.create_service_client(self.node, SwitchController, self.switch_service_name)

        self.node.get_logger().warning("END Init ControllersConnection")

    def switch_controllers(self, controllers_on, controllers_off, strictness=1):
        """
        Give the controllers you want to switch on or off.
        :param controllers_on: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :param controllers_off: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        # rospy.wait_for_service(self.switch_service_name) # Already waited for service in constructor

        try:
            switch_request_object = SwitchController.Request()
            switch_request_object.start_controllers = controllers_on
            switch_request_object.start_controllers = controllers_off
            switch_request_object.strictness = strictness

            # switch_result = self.switch_service(switch_request_object)
            switch_result = service_utils.call_and_wait_for_service_response(self.node, self.switch_service_client, switch_request_object)
            """
            [controller_manager_msgs/SwitchController]
            int32 BEST_EFFORT=1
            int32 STRICT=2
            string[] start_controllers
            string[] stop_controllers
            int32 strictness
            ---
            bool ok
            """
            self.node.get_logger().debug("Switch Result==>"+str(switch_result.ok))

            return switch_result.ok

        except exceptions.SpinFutureTimeoutException as e:
            print (self.switch_service_name+" service call failed")

            return None

    def reset_controllers(self):
        """
        We turn on and off the given controllers
        :param controllers_reset: ["name_controler_1", "name_controller2",...,"name_controller_n"]
        :return:
        """
        reset_result = False

        result_off_ok = self.switch_controllers(controllers_on = [],
                                controllers_off = self.controllers_list)

        self.node.get_logger().debug("Deactivated Controlers")

        if result_off_ok:
            self.node.get_logger().debug("Activating Controlers")
            result_on_ok = self.switch_controllers(controllers_on=self.controllers_list,
                                                    controllers_off=[])
            if result_on_ok:
                self.node.get_logger().debug("Controllers Reseted==>"+str(self.controllers_list))
                reset_result = True
            else:
                self.node.get_logger().debug("result_on_ok==>" + str(result_on_ok))
        else:
            self.node.get_logger().debug("result_off_ok==>" + str(result_off_ok))

        return reset_result

    def update_controllers_list(self, new_controllers_list):
        self.controllers_list = new_controllers_list
