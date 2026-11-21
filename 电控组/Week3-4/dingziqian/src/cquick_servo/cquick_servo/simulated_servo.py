class SimulatedServoBus:
    def __init__(self, servo_ids, initial_pwm):
        self.positions = {}
        self.torque_disabled_ids = set()

        for servo_id in servo_ids:
            self.positions[servo_id] = initial_pwm

    def read_position(self, servo_id):
        if servo_id not in self.positions:
            raise ValueError(f"不存在的舵机 ID：{servo_id}")

        return self.positions[servo_id]

    def move_to(self, servo_id, target_pwm):
        if servo_id not in self.positions:
            raise ValueError(f"不存在的舵机 ID: {servo_id}")

        self.positions[servo_id] = target_pwm
        self.torque_disabled_ids.discard(servo_id)

    def disable_torque(self, servo_id):
        if servo_id not in self.positions:
            raise ValueError(f"不存在的舵机 ID: {servo_id}")
        self.torque_disabled_ids.add(servo_id)


if __name__ == "__main__":
    bus = SimulatedServoBus([1, 2], 1500)

    print("1 号模拟位置：", bus.read_position(1))
    print("2 号模拟位置：", bus.read_position(2))

    bus.move_to(1, 1700)

    print("移动后，1 号模拟位置：", bus.read_position(1))
    print("移动后，2 号模拟位置：", bus.read_position(2))
