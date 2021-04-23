import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.timer import Rate

class RateRunner():
    def __init__(self, node: Node, frequency, args, executor=None):
        self.done = False
        rate = node.create_rate(frequency)

        # self._thread = threading.Thread(target=self._run, args=(rate, func, cond), daemon=True)
        self._thread = threading.Thread(target=self._run, args=(rate,)+args, daemon=True)
        self._thread.start()

        if executor is None:
            executor = SingleThreadedExecutor(context=rclpy.get_default_context())
        executor.add_node(node)

        while not self.done:
            executor.spin_once()
        rate.destroy()
        executor.remove_node(node)
    
    def _run(self):
        raise NotImplementedError


class WhileRateRunner(RateRunner):
    def __init__(self, node: Node, frequency, func=None, cond=lambda: True, executor=None):
        super().__init__(node, frequency, (func, cond), executor)

    def _run(self, rate, func, cond):
        try:
            while cond() and rclpy.ok():
                if func is not None:
                    func()
                rate.sleep()
        finally:
            self.done = True

class ForRateRunner(RateRunner):
    def __init__(self, node: Node, frequency, func=None, num_loops=1, executor=None):
        super().__init__(node, frequency, (func, num_loops), executor)

    def _run(self, rate, func, num_loops):
        try:
            for _ in range(num_loops):
                if rclpy.ok():
                    if func is not None:
                        func()
                    rate.sleep()
        finally:
            self.done = True