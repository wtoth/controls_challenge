from . import BaseController

class Controller(BaseController):
  """
  Basic PidController Implementation - Current best vals are (0.08, 1.0, 0.0)
  """
  def __init__(self, kp=0.085, ki=1.1, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

  def update(self, target_lataccel, current_lataccel, state, future_plan):
    dt = 0.1
    smoothing_term = 0.9
    error = target_lataccel - current_lataccel
    self.integral += error * dt
    derivative = (error - self.previous_error) / dt

    #0.9 is a regularization term aimed at lowering jerk_cost
    steer_action = (self.kp * error + self.ki * self.integral + self.kd * derivative)*smoothing_term
    self.previous_error = error

    return steer_action

# vEgo	aEgo	roll	targetLateralAcceleration	steerCommand
# target_lataccel, current_lataccel, state
# -0.0176365489509296 -0.0159983567850293 State(roll_lataccel=-0.11380118726440701, v_ego=33.63961519611496, a_ego=0.0433645300577389)
