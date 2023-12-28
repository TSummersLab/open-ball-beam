import math

import gym
import numpy as np
from gym import logger, spaces
from gym.utils import seeding

from ballbeam.common.colors import Monokai
from ballbeam.common.cost import Cost
from ballbeam.common.hardware import Hardware
from ballbeam.common.reference import ConstantReference
from ballbeam.common.simulator import XMAX, XMIN, YMAX, YMIN, Simulator
from ballbeam.configurators.configs import CONFIG


class BallBeamEnv(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

    def __init__(self, max_episode_length=200, hardware=False, seed=None) -> None:
        self.seed(seed)
        self.viewer = None
        self.umin = CONFIG.hardware.BEAM.ANGLE.MIN * CONFIG.constants.DEG2RAD
        self.umax = CONFIG.hardware.BEAM.ANGLE.MAX * CONFIG.constants.DEG2RAD
        self.action_space = None
        # VMAX = (YMAX-YMIN)/CONFIG.hardware.COMM.DT
        # VMIN = -VMAX
        # obs_lo, obs_hi = np.array([YMIN, VMIN], dtype=np.float32), np.array([YMAX, VMAX], dtype=np.float32)
        obs_lo, obs_hi = np.array(YMIN, dtype=np.float32), np.array(YMAX, dtype=np.float32)
        self.observation_space = spaces.Box(obs_lo, obs_hi, dtype=np.float32)
        self.max_episode_length = max_episode_length
        if hardware:
            self.system = Hardware()
        else:
            self.system = Simulator()
            # self.system = Simulator(process_noise_scale=0.1, sensor_noise_scale=0.1)
        self.reference = ConstantReference()
        self.cost = Cost()
        self.reset()

    def seed(self, seed=None):
        self.np_random, _seed = seeding.np_random(seed)
        np.random.seed(seed)
        return [seed]

    def action2theta(self, action) -> None:
        raise NotImplementedError

    def step(self, action):
        # This function mirrors update() in interface.py

        theta = self.action2theta(action)
        theta = np.clip(theta, self.umin, self.umax)
        self.theta = theta

        # Get a new observation from the system
        self.observation = self.system.observe()

        # Get the current setpoint from the reference
        self.setpoint = self.reference.setpoint(self.t)

        # # Update the controller with latest information
        # controller.update_aux(saturated=self.system.saturated, ball_removed=self.system.ball_removed)
        # controller.update(observation, setpoint, self.t)

        # # Get the control action from the controller based on latest information
        # action = controller.action

        # # Get the current estimate of the state from the controller
        # state_estimate = controller.state_estimate

        # Get the current cost of the observation and theta
        cost_dict = self.cost.take(self.observation, theta)
        reward = -cost_dict["c"]

        # Evolve the system forward by one step
        self.system.process(theta)

        # done = bool(not XMIN+0.010 < self.observation < XMAX-0.010)  # use 0.99 fudge factor so simulator doesnt have to be altered
        done = bool(self.t + 1 >= self.max_episode_length)
        # done = False  # never terminate early

        # Increment the time index
        self.t += 1

        if not done:
            pass
        elif self.steps_beyond_done is None:
            # just failed
            self.steps_beyond_done = 0
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior.",
                )
            self.steps_beyond_done += 1
            reward = 0.0

        info = {}

        return self.observation, reward, done, info

    def reset(self, pos0=None):
        if pos0 is None:
            # pos0 = self.np_random.uniform(XMIN+0.020, XMAX-0.020)
            # pos0 = self.np_random.choice([XMIN+0.020, XMIN+0.040, XMAX-0.040, XMAX-0.020])
            pos0 = XMIN + 0.020
        self.system.reset(np.array([pos0, 0.0]))
        self.theta = 0.0
        self.t = 0
        self.setpoint = self.reference.setpoint(self.t)
        self.observation = pos0
        self.saturated = False
        self.steps_beyond_done = None
        return self.observation

    def render(self, mode="human"):
        screen_width = 600
        screen_height = 200

        world_width = (XMAX - XMIN) * 1.25
        scale = screen_width / world_width
        axlex, axley = 0.9 * screen_width, 0.3 * screen_height
        beamlength = scale * (XMAX - XMIN)
        beamthick = 20
        ballrad = 40
        tgtballrad = 44
        axlerad = beamthick

        if self.viewer is None:
            from gym.envs.classic_control import rendering

            self.viewer = rendering.Viewer(screen_width, screen_height)

            # Background
            color = tuple(val / 255 for val in Monokai.k)
            l, r, t, b = 0, screen_width, screen_height, 0
            bkgd = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            bkgd.set_color(*color)
            self.viewer.add_geom(bkgd)

            self.axletrans = rendering.Transform()
            self.axletrans.set_translation(axlex, axley)

            # Add the beam
            l, r, t, b = -beamlength / 2, beamlength / 2, beamthick / 2, -beamthick / 2
            self.beam = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            color = tuple(val / 255 for val in Monokai.b)
            self.beam.set_color(*color)
            self.beamtrans = rendering.Transform()
            self.beam.add_attr(self.beamtrans)
            self.beam.add_attr(self.axletrans)
            self.viewer.add_geom(self.beam)

            # Add the axle
            self.axle = rendering.make_circle(radius=axlerad)
            self.axle.add_attr(self.axletrans)
            color = tuple(val / 255 for val in Monokai.s)
            self.axle.set_color(*color)
            self.viewer.add_geom(self.axle)

            # Add the setpoint marker
            self.tgtball = rendering.make_circle(radius=tgtballrad, filled=False)
            self.tgtball.set_linewidth(2)
            self.tgtballtrans = rendering.Transform()
            self.tgtball.add_attr(self.tgtballtrans)
            self.tgtball.add_attr(self.axletrans)
            color = tuple(val / 255 for val in Monokai.y)
            self.tgtball.set_color(*color)
            self.viewer.add_geom(self.tgtball)

            # Add the ball
            self.ball = rendering.make_circle(radius=ballrad, filled=True)
            self.balltrans = rendering.Transform()
            self.ball.add_attr(self.balltrans)
            self.ball.add_attr(self.axletrans)
            color = tuple(val / 255 for val in Monokai.w)
            self.ball.set_color(*color)
            self.viewer.add_geom(self.ball)

        if self.observation is None:
            return None

        # Dynamic updates
        x = self.observation + self.setpoint
        beamx = -math.cos(self.theta) * 0.5 * beamlength
        beamy = -math.sin(self.theta) * 0.5 * beamlength

        ballx = -math.cos(self.theta) * ((XMAX - XMIN) - (x - XMIN)) * scale
        bally = -math.sin(self.theta) * ((XMAX - XMIN) - (x - XMIN)) * scale + beamthick / 2.0 + ballrad

        self.balltrans.set_translation(ballx, bally)
        self.beamtrans.set_rotation(self.theta)
        self.beamtrans.set_translation(beamx, beamy)

        x_tgt = self.setpoint
        tgtx = -math.cos(self.theta) * ((XMAX - XMIN) - (x_tgt - XMIN)) * scale
        tgty = -math.sin(self.theta) * ((XMAX - XMIN) - (x_tgt - XMIN)) * scale + beamthick / 2.0 + ballrad
        self.tgtballtrans.set_translation(tgtx, tgty)

        return_rgb_array = mode == "rgb_array"

        return self.viewer.render(return_rgb_array=return_rgb_array)

    def close(self) -> None:
        if self.viewer:
            self.viewer.close()
            self.viewer = None
        self.system.shutdown()


class BallBeamContinuousEnv(BallBeamEnv):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.action_space = spaces.Box(self.umin, self.umax, shape=(1,), dtype=np.float32)

    def action2theta(self, action):
        return action


class BallBeamDiscreteEnv(BallBeamEnv):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.action_space = spaces.Discrete(3)

    def action2theta(self, action):
        if action == 2:
            return 3 * CONFIG.constants.DEG2RAD
        elif action == 0:
            return -3 * CONFIG.constants.DEG2RAD
        else:
            return 0.0


if __name__ == "__main__":
    from time import time

    from ballbeam.common.interface import choose_controller

    use_discrete_env = False
    use_hardware = False
    max_episode_length = 400
    seed = None

    # Instantiate the Gym environment
    Env = BallBeamDiscreteEnv if use_discrete_env else BallBeamContinuousEnv
    env = Env(hardware=use_hardware, max_episode_length=max_episode_length)
    env.seed(seed)  # TODO harmonize random seeding of the simulator in simulator.py and Gym

    # Choose controller
    # controller_type = 'Null'
    # controller_type = 'Sine'
    # controller_type = 'PID'
    controller_type = "LQG"
    # controller_type = 'MPC'
    controller = choose_controller(controller_type)

    # Initialize
    action = 0
    time_elapsed = 0
    time_last = time()
    reward_tot = 0
    observation = env.reset()

    # Run an episode
    for i in range(max_episode_length):
        time_now = time()
        env.render()

        # Update the controller with latest information
        controller.update_aux(saturated=env.system.saturated, ball_removed=env.system.ball_removed)
        controller.update(observation, env.setpoint, env.t)

        # Get the control action from the controller based on latest information
        cont_action = controller.action

        if use_discrete_env:
            if np.abs(cont_action) > 0.02:  # Only act if the requested cont_action is large
                action = int(np.sign(cont_action) + 1)
            else:
                action = 1
        else:
            action = cont_action

        observation, reward, done, info = env.step(action)
        reward_tot += reward

        spacer = "    "
        print(i + 1, end="    ")
        print("%12.6f" % observation, end=spacer)
        print("%12.6f" % reward, end=spacer)
        print("%12.6f" % reward_tot, end=spacer)
        print("%12.6f" % (time_now - time_last), end=spacer)
        print("")

        time_last = time_now
        if done:
            break
    env.close()
