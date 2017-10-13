
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 0.

    def control_speed_based_on_proportional_throttle_brake(target_velocity,
        current_linear_velocity, max_throttle, max_brake):

        return 0.8, 0
