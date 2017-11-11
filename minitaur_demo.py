import math
import minitaur
import time

NUM_DEMOS = 3
EACH_DEMO_DURATION = 5

class MinitaurDemo():
  def __init__(self):
    self.minitaur = minitaur.Minitaur()
    self.reset_time = time.time()
    self.demo_list = [self._run, self._run_backward, self._squat]
    self.t = 0
  def reset(self):
    self.minitaur.Reset(reload_urdf=False)
    self.reset_time = time.time()

  def update(self, context):
    current_time = time.time()
    time_since_reset = current_time - self.reset_time
    demo_index = int(time_since_reset / EACH_DEMO_DURATION) % NUM_DEMOS
    delta_time = time_since_reset
    if (context.vrMode):
      self.t=self.t+0.003
      if (self.t>3.141592):
        self.t = self.t-3.141592
      delta_time = self.t
    self.demo_list[demo_index](delta_time)

  def _squat(self, time):
    action = 0.5 * math.sin(3 * time) + math.pi / 2
    actions = [action] * 8
    self.minitaur.ApplyAction(actions)

  def _run(self, time):
    params = [0.1903581461951056, 0.0006732219568880068, 0.05018085615283363, 3.219916795483583, 6.2406418167980595, 4.189869754607539]  # Optimized using Bayesian optimization
    speed = 35.0
    phaseDiff = params[2]
    a0 = math.sin(time* speed) * params[0] + 1.57
    a1 = math.sin(time* speed + phaseDiff) * params[1] + 1.57
    a2 = math.sin(time* speed + params[3]) * params[0] + 1.57
    a3 = math.sin(time* speed + params[3] + phaseDiff) * params[1] + 1.57
    a4 = math.sin(time* speed + params[4] + phaseDiff) * params[1] + 1.57
    a5 = math.sin(time* speed + params[4]) * params[0] + 1.57
    a6 = math.sin(time* speed + params[5] + phaseDiff) * params[1] + 1.57
    a7 = math.sin(time* speed + params[5]) * params[0] + 1.57
    actions = [a0, a1, a2, a3, a4, a5, a6, a7]
    self.minitaur.ApplyAction(actions)

  def _run_backward(self, time):
    self._run(-time)
