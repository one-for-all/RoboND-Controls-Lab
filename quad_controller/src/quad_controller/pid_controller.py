# Copyright (C) 2017 Udacity Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10, alpha=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_windup = max_windup
        self.target = 0
        self.alpha = alpha

        # memory variables for PID computation
        self.error_sum = 0
        self.last_timestamp = 0
        self.last_error = 0

        # memory variable for noise smoothing
        self.last_d = None

    def reset(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.max_windup = 0
        self.target = 0
        self.error_sum = 0
        self.last_timestamp = 0
        self.last_error = 0
        self.last_d = None

    def setTarget(self, target):
        self.target = target

    def setKP(self, kp):
        self.kp = kp

    def setKI(self, ki):
        self.ki = ki

    def setKD(self, kd):
        self.kd = kd

    def setMaxWindup(self, max_windup):
        self.max_windup = max_windup

    def update(self, measured_value, timestamp):
        # time difference used for integration and differentiation
        delta_time = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        # Compute p
        error = self.target - measured_value
        p = self.kp * error

        # compute i
        self.error_sum += error * delta_time
        # restrict to max_windup for stability
        if self.error_sum > self.max_windup:
            self.error_sum = self.max_windup
        elif self.error_sum < -self.max_windup:
            self.error_sum = -self.max_windup
        i = self.ki * self.error_sum

        # compute d
        delta_error = error - self.last_error
        self.last_error = error
        new_d = self.kd * delta_error/delta_time
        if self.last_d is None:
            d = new_d
        else:
            d = self.alpha * new_d + (1-self.alpha) * self.last_d
        self.last_d = d

        # compute output
        u = p + i + d
        return u