#!/usr/bin/env python3
"""
Bring the modelidar board up when it is there, and back up after an unplug or an error.

The controller manager starts with the hardware component UNCONFIGURED (parameter
hardware_components_initial_state): it no longer aborts when the board is missing.
Every `period` seconds this node:
  - activates the hardware component if it is not active and the serial device exists
    (an error, e.g. board unplugged, sends the component back to UNCONFIGURED);
  - then activates the controllers of `controllers` that are loaded but inactive (the
    controller manager deactivates them when their hardware fails).

A controller deactivated by hand is therefore activated again: stop this node first.
"""

import os
import time

from controller_manager_msgs.srv import (
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from lifecycle_msgs.msg import State
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class BoardSupervisor(Node):
    """Keep the hardware component and its controllers active."""

    def __init__(self):
        super().__init__('board_supervisor')
        self.hardware = self.declare_parameter('hardware', 'Modelidar').value
        self.device = self.declare_parameter('device', '').value  # '' = do not wait for it
        self.controllers = list(self.declare_parameter(
            'controllers', ['joint_state_broadcaster', 'modelidar_base_controller']).value)
        self.period = self.declare_parameter('period', 1.0).value
        self.call_timeout = self.declare_parameter('call_timeout', 5.0).value
        cm = self.declare_parameter('controller_manager', '/controller_manager').value

        self.list_hw = self.create_client(ListHardwareComponents, cm + '/list_hardware_components')
        self.set_hw = self.create_client(
            SetHardwareComponentState, cm + '/set_hardware_component_state')
        self.list_ctrl = self.create_client(ListControllers, cm + '/list_controllers')
        self.switch_ctrl = self.create_client(SwitchController, cm + '/switch_controller')

        self.last_status = None  # logged once per change, to keep the log readable

    def status(self, message, error=False):
        """Log `message` only when it differs from the previous status."""
        if message != self.last_status:
            (self.get_logger().error if error else self.get_logger().info)(message)
            self.last_status = message

    def call(self, client, request):
        """Call a service and wait for the answer. None if unavailable or too slow."""
        if not client.wait_for_service(timeout_sec=self.call_timeout):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout)
        return future.result() if future.done() else None

    def hardware_state(self):
        """Lifecycle state id of the hardware component, None if not known yet."""
        answer = self.call(self.list_hw, ListHardwareComponents.Request())
        if answer is None:
            self.status('Waiting for the controller manager')
            return None
        for component in answer.component:
            if component.name == self.hardware:
                return component.state.id
        self.status(f"Waiting for hardware component '{self.hardware}' (robot description)")
        return None

    def activate_hardware(self):
        if self.device and not os.path.exists(self.device):
            self.status(f'Waiting for {self.device} (board unplugged?)')
            return
        request = SetHardwareComponentState.Request()
        request.name = self.hardware
        request.target_state = State(id=State.PRIMARY_STATE_ACTIVE, label='active')
        answer = self.call(self.set_hw, request)
        if answer is not None and answer.ok:
            self.status(f"Hardware '{self.hardware}' active")
        else:
            self.status(
                f"Could not activate '{self.hardware}' (board not answering?), retrying",
                error=True)

    def activate_controllers(self):
        answer = self.call(self.list_ctrl, ListControllers.Request())
        if answer is None:
            return
        inactive = [
            c.name for c in answer.controller
            if c.name in self.controllers and c.state == 'inactive']
        if not inactive:
            return
        request = SwitchController.Request()
        request.activate_controllers = inactive
        request.strictness = SwitchController.Request.BEST_EFFORT
        answer = self.call(self.switch_ctrl, request)
        if answer is not None and answer.ok:
            self.get_logger().info(f"Controllers activated: {', '.join(inactive)}")
        else:
            self.status(f"Could not activate {', '.join(inactive)}, retrying", error=True)

    def step(self):
        state = self.hardware_state()
        if state is None:
            return
        if state == State.PRIMARY_STATE_ACTIVE:
            self.activate_controllers()
        elif state in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE):
            self.activate_hardware()
        elif state == State.PRIMARY_STATE_FINALIZED:
            self.status(f"Hardware '{self.hardware}' finalized: restart the launch", error=True)


def main(args=None):
    rclpy.init(args=args)
    node = BoardSupervisor()
    try:
        while rclpy.ok():
            node.step()
            time.sleep(node.period)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
