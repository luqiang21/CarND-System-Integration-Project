
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        pass

    def control(self, cte, dbw_enabled, twist_cmd_linear_velocity,
                twist_cmd_angular_velocity, current_linear_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 0.0

    def control_speed_based_on_proportional_throttle_brake(self, target_velocity,
        current_linear_velocity, max_throttle, max_brake):

        return 0.8, 0.0
