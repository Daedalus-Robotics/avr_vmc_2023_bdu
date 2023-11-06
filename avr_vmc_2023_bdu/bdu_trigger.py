from threading import Event
from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from avr_pcc_2023_interfaces.srv import SetServo


class BDUTriggerNode(Node):
    def __init__(self) -> None:
        super().__init__('bdu_trigger', namespace='bdu')

        self.declare_parameter('hold_duration', 1000)
        self.declare_parameter('max_value', 175)
        self.declare_parameter('min_value', 0)
        self.declare_parameter('servo_num', 0)

        self.hold_duration = self.get_parameter('hold_duration').value
        self.max_value = self.get_parameter('max_value').value
        self.min_value = self.get_parameter('min_value').value
        self.servo_num = self.get_parameter('servo_num').value

        # noinspection PyTypeChecker
        self.trigger_service = self.create_service(
                Trigger,
                'trigger',
                self.trigger
        )
        self.set_servo_client = self.create_client(
                SetServo,
                '/servo/set_position',
        )

        self.finish_timer = self.create_timer(
                callback=self.finish,
                timer_period_sec=self.hold_duration,
        )
        self.finish_timer.cancel()

        setup_done = False
        while not setup_done:
            self.get_logger().error('No response from servo service. Retrying...', skip_first=True)
            future = self.set_servo(False)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2)
            setup_done = True
            # setup_done = future.done()
            # if not setup_done:
            #     future.cancel()

    def trigger(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """
        The trigger service callback.
        Calls the servo service to drop, then starts a timer to finish the drop.

        :param _: The request object.
        This parameter is ignored as it has no data.
        :param response: The response object to be updated based on the servo service response.

        :return: The updated response object.
        """
        future_done = Event()
        future = self.set_servo(True)
        # future.add_done_callback(lambda: future_done.set())
        future_done.set()

        if future_done.wait(1):
            # servo_response = future.result()

            # response.success = servo_response.success
            # response.message = 'Success' if servo_response.success else 'Failed'

            self.finish_timer.reset()
            self.get_logger().info('Reset the timer')
        else:
            # future.cancel()

            response.success = False
            response.message = 'No response from servo service'
            self.get_logger().warn('Failed to call service to start drop')

        return response

    def finish(self) -> None:
        """
        This method is used to finish the task by setting the servo to False, canceling the finish timer,
        and handling any timeouts that may occur.
        """
        future_done = Event()
        future = self.set_servo(False)
        # future.add_done_callback(lambda: future_done.set())
        future_done.set()

        if future_done.wait(1):
            # servo_response = future.result()

            # if not servo_response.success:
            #     self.get_logger().warn('Failed set servo to finish drop. Retrying...')
            # else:
            self.finish_timer.cancel()
            self.get_logger().info('Cancel the timer')
        else:
            # future.cancel()

            self.get_logger().warn('Failed to call service to finish drop. Retrying...')

    def set_servo(self, state: bool) -> rclpy.Future:
        """
        Sends a request to set the servo position based on the provided state.

        :param state: A boolean indicating whether the servo should be set to the maximum value or the minimum value.

        :return: A future that represents the asynchronous result of the request.
        """
        servo_request = SetServo.Request()
        servo_request.servo_num = self.servo_num
        servo_request.value = self.max_value if state else self.min_value
        return self.set_servo_client.call_async(servo_request)


def main() -> None:
    rclpy.init()
    node = BDUTriggerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)


if __name__ == '__main__':
    main()
