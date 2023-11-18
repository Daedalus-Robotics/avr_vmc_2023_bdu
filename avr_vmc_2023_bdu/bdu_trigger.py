from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from avr_pcc_2023_interfaces.srv import SetServo


class BDUTriggerNode(Node):
    def __init__(self) -> None:
        super().__init__('bdu_trigger', namespace='bdu')

        self.declare_parameter('hold_duration', 1.0)
        self.declare_parameter('stage', 50)
        self.declare_parameter('stage_count', 4)
        self.declare_parameter('min_value', 0)
        self.declare_parameter('servo_num', 0)

        self.hold_duration = self.get_parameter('hold_duration').value
        self.stage_length = int(self.get_parameter('stage_length').value)
        self.stage_count = int(self.get_parameter('stage_count').value)
        self.min_value = int(self.get_parameter('min_value').value)
        self.servo_num = int(self.get_parameter('servo_num').value)

        self.full_trigger_service = self.create_service(
                Trigger,
                'full_trigger',
                self.full_trigger
        )
        self.trigger_service = self.create_service(
                Trigger,
                'trigger',
                self.trigger
        )
        self.reset_service = self.create_service(
                Trigger,
                'reset',
                self.reset
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

        self.get_logger().error('Waiting for servo set_position service...')
        self.set_servo_client.wait_for_service()
        future = self.set_servo(False)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        self.prev_future: rclpy.Future = rclpy.Future()
        self.current_stage: int = 0

    def full_trigger(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.prev_future.cancel()
        self.current_stage = self.stage_count + 1

        future = self.set_servo(self.current_stage)
        self.prev_future = future
        future.add_done_callback(lambda _: self.finish_timer.reset())

        response.message = 'full triggered. Resetting...'
        response.success = True
        return response

    def trigger(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.prev_future.cancel()
        self.current_stage = (self.current_stage + 1) % (self.stage_count + 1)

        future = self.set_servo(self.current_stage)
        self.prev_future = future
        if self.current_stage == self.stage_count:
            future.add_done_callback(lambda _: self.finish_timer.reset())
            response.message = 'Last stage triggered. Resetting...'
        else:
            response.message = 'Triggered stage'

        response.success = True
        return response

    def reset(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.prev_future.cancel()
        self.finish_timer.cancel()

        self.current_stage = 0

        self.prev_future.cancel()
        self.prev_future = self.set_servo(0)

        response.success = True
        response.message = 'Reset'
        return response

    def finish(self) -> None:
        self.prev_future.cancel()
        self.prev_future = self.set_servo(0)

    def set_servo(self, state: int) -> rclpy.Future:
        state = min(self.stage_count + 1, max(0, state))

        servo_request = SetServo.Request()
        servo_request.servo_num = self.servo_num
        servo_request.value = (self.stage_length * state) + self.min_value
        return self.set_servo_client.call_async(servo_request)


def main() -> None:
    rclpy.init()
    node = BDUTriggerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)


if __name__ == '__main__':
    main()
