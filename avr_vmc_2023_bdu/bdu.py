import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from avr_pcc_2023_interfaces.srv import SetServo


class BDUNode(Node):
    def __init__(self) -> None:
        super().__init__('bdu', namespace="bdu")

        self.declare_parameter('hold_duration', 1000)
        self.declare_parameter('max_value', 175)
        self.declare_parameter('min_value', 0)
        self.declare_parameter('servo_num', 0)

        self.hold_duration = self.get_parameter('hold_duration').value
        self.max_value = self.get_parameter('max_value').value
        self.min_value = self.get_parameter('min_value').value
        self.servo_num = self.get_parameter('servo_num').value

        self.set_servo(False)

        self.trigger_service = self.create_service(
            Trigger,
            'trigger',
            self.trigger
        )
        self.set_servo_client = self.create_client(
            SetServo,
            '/pcc/set_position',
        )

        self.finish_timer = self.create_timer(
            callback=self.finish,
            timer_period_sec=self.hold_duration,
        )
        self.finish_timer.cancel()

    def trigger(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.finish_timer.cancel()
        servo_response = self.set_servo(True)

        response.success = servo_response.success
        response.message = 'Success' if servo_response.success else 'Failed'

        self.finish_timer.reset()

        return response

    def finish(self) -> None:
        self.finish_timer.cancel()
        self.set_servo(False)

    def set_servo(self, state: bool) -> SetServo.Response:
        servo_request = SetServo.Request()
        servo_request.servo_num = self.servo_num
        servo_request.value = self.max_value if state else self.min_value
        return self.set_servo_client.call(servo_request)


def main() -> None:
    rclpy.init()
    node = BDUNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
