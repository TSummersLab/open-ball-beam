import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

from settings import DEG2RAD, DT
from colors import Monokai

from reference import ConstantReference, PeriodicReference
from cost import Cost
from simulator import Simulator, XMIN, XMAX, YMIN, YMAX
from hardware import Hardware

from interface import action2actuation


class BallBeamEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self, max_episode_length=200, hardware=False):
        self.seed()
        self.viewer = None

        self.action_space = spaces.Box(-5*DEG2RAD, 5*DEG2RAD, shape=(1, ), dtype=np.float32)
        VMAX = (YMAX-YMIN)/DT
        VMIN = -VMAX
        obs_lo, obs_hi = np.array([YMIN, VMIN], dtype=np.float32), np.array([YMAX, VMAX], dtype=np.float32)
        self.observation_space = spaces.Box(obs_lo, obs_hi, dtype=np.float32)
        self.max_episode_length = max_episode_length
        if hardware:
            self.system = Hardware()
        else:
            self.system = Simulator()
        self.reference = ConstantReference()
        self.cost = Cost()
        self.reset()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        action = np.float32(action)[0]
        # err_msg = "%r (%s) invalid"%(action, type(action))
        # assert self.action_space.contains(action), err_msg  # redundant due to above 2 lines
        self.theta = action

        actuation, self.saturated = action2actuation(action, ball_removed=False)
        self.setpoint = self.reference.setpoint(self.t)
        self.system.process(action, actuation)
        new_err_obs = self.system.observe() - self.setpoint
        self.vel_est = (new_err_obs - self.err_est)/DT
        self.err_est = new_err_obs
        self.obs = np.array([self.err_est, self.vel_est])
        cost_dict = self.cost.take(self.err_est, action)
        reward = -cost_dict['c']
        # done = bool(not XMIN+0.010 < self.obs < XMAX-0.010)  # use 0.99 fudge factor so simulator doesnt have to be altered
        done = bool(self.t+1 >= self.max_episode_length)
        # done = False  # never terminate early
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
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0

        return self.obs, reward, done, {}

    def reset(self, pos0=None):
        if pos0 is None:
            pos0 = self.np_random.uniform(XMIN+0.020, XMAX-0.020)
            # pos0 = self.np_random.choice([XMIN+0.020, XMIN+0.040, XMAX-0.040, XMAX-0.020])
            # pos0 = XMIN+0.020
        self.system.reset(np.array([pos0, 0.0]))
        self.theta = 0.0
        self.t = 0
        self.setpoint = self.reference.setpoint(self.t)
        self.err_est = 0.0
        self.vel_est = 0.0
        self.obs = np.array([self.err_est, self.vel_est])
        self.saturated = False
        self.steps_beyond_done = None
        return self.obs

    def render(self, mode='human'):
        screen_width = 600
        screen_height = 200

        world_width = (XMAX-XMIN) * 1.25
        scale = screen_width/world_width
        axlex, axley = 0.9*screen_width, 0.3*screen_height
        beamlength = scale * (XMAX-XMIN)
        beamthick = 20
        ballrad = 40
        tgtballrad = 44
        axlerad = beamthick

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            # Background
            color = tuple(val/255 for val in Monokai.k)
            l, r, t, b = 0, screen_width, screen_height, 0
            bkgd = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            bkgd.set_color(*color)
            self.viewer.add_geom(bkgd)

            self.axletrans = rendering.Transform()
            self.axletrans.set_translation(axlex, axley)

            # Add the beam
            l, r, t, b = -beamlength/2, beamlength/2, beamthick/2, -beamthick/2
            self.beam = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            color = tuple(val/255 for val in Monokai.b)
            self.beam.set_color(*color)
            self.beamtrans = rendering.Transform()
            self.beam.add_attr(self.beamtrans)
            self.beam.add_attr(self.axletrans)
            self.viewer.add_geom(self.beam)

            # Add the axle
            self.axle = rendering.make_circle(radius=axlerad)
            self.axle.add_attr(self.axletrans)
            color = tuple(val/255 for val in Monokai.s)
            self.axle.set_color(*color)
            self.viewer.add_geom(self.axle)

            # Add the setpoint marker
            self.tgtball = rendering.make_circle(radius=tgtballrad, filled=False)
            self.tgtball.set_linewidth(2)
            self.tgtballtrans = rendering.Transform()
            self.tgtball.add_attr(self.tgtballtrans)
            self.tgtball.add_attr(self.axletrans)
            color = tuple(val/255 for val in Monokai.y)
            self.tgtball.set_color(*color)
            self.viewer.add_geom(self.tgtball)

            # Add the ball
            self.ball = rendering.make_circle(radius=ballrad, filled=True)
            self.balltrans = rendering.Transform()
            self.ball.add_attr(self.balltrans)
            self.ball.add_attr(self.axletrans)
            color = tuple(val/255 for val in Monokai.w)
            self.ball.set_color(*color)
            self.viewer.add_geom(self.ball)

        if self.obs is None:
            return None

        # Dynamic updates
        x = self.obs[0] + self.setpoint
        beamx = -math.cos(self.theta)*0.5*beamlength
        beamy = -math.sin(self.theta)*0.5*beamlength

        ballx = -math.cos(self.theta)*((XMAX-XMIN)-(x-XMIN))*scale
        bally = -math.sin(self.theta)*((XMAX-XMIN)-(x-XMIN))*scale + beamthick/2.0 + ballrad

        self.balltrans.set_translation(ballx, bally)
        self.beamtrans.set_rotation(self.theta)
        self.beamtrans.set_translation(beamx, beamy)

        x_tgt = self.setpoint
        tgtx = -math.cos(self.theta)*((XMAX-XMIN)-(x_tgt-XMIN))*scale
        tgty = -math.sin(self.theta)*((XMAX-XMIN)-(x_tgt-XMIN))*scale + beamthick/2.0 + ballrad
        self.tgtballtrans.set_translation(tgtx, tgty)

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None
        self.system.shutdown()




# TODO make BallBeamDiscreteEnv a child class of BallBeamEnv and just replace method code as needed
# TODO make pos_est the position error like BallBeamEnv
class BallBeamDiscreteEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self.seed()
        self.viewer = None
    #
    #     self.action_space = spaces.Discrete(2)
    #     # self.observation_space = spaces.Box(YMIN, YMAX, shape=(1,), dtype=np.float32)
    #     VMAX = (YMAX-YMIN)/DT
    #     VMIN = -VMAX
    #     obs_lo, obs_hi = np.array([YMIN, VMIN], dtype=np.float32), np.array([YMAX, VMAX], dtype=np.float32)
    #     self.observation_space = spaces.Box(obs_lo, obs_hi, dtype=np.float32)
    #
    #     self.system = Simulator()
    #     # self.system = Hardware()
    #
    #     self.reference = ConstantReference()
    #
    #     self.cost = Cost()
    #
    #     self.reset()
    #
    # def seed(self, seed=None):
    #     self.np_random, seed = seeding.np_random(seed)
    #     return [seed]
    #
    # def step(self, action):
    #     err_msg = "%r (%s) invalid"%(action, type(action))
    #     assert self.action_space.contains(action), err_msg  # redundant due to above 2 lines
    #     self.theta = 3*DEG2RAD if action == 1 else -3*DEG2RAD
    #
    #     actuation, self.saturated = action2actuation(self.theta, ball_removed=False)
    #     self.setpoint = self.reference.setpoint(self.t)
    #     self.system.process(self.theta, actuation)
    #     new_pos_obs = self.system.observe()
    #     self.vel_est = (new_pos_obs - self.pos_est)/DT
    #     self.pos_est = new_pos_obs
    #     self.obs = np.array([self.pos_est, self.vel_est])
    #     # reward_alive = 1.0
    #     # reward_error = -(100.0*abs(self.pos_est-self.setpoint))
    #
    #     # # v1
    #     # reward = reward_alive
    #     # done = bool(not XMIN+0.010 < self.pos_est < XMAX-0.010) or bool(self.t+1 >= 200)  # use 0.99 fudge factor so simulator doesnt have to be altered
    #
    #     # # v2
    #     # reward = reward_error
    #     # done = bool(self.t+1 >= 200)
    #
    #     cost_dict = self.cost.take(self.pos_est, action)
    #     reward = -cost_dict['c']
    #     done = bool(self.t + 1 >= 200)
    #
    #     # done = False  # never terminate early
    #     self.t += 1
    #
    #     if not done:
    #         pass
    #     elif self.steps_beyond_done is None:
    #         # just failed
    #         self.steps_beyond_done = 0
    #     else:
    #         if self.steps_beyond_done == 0:
    #             logger.warn(
    #                 "You are calling 'step()' even though this "
    #                 "environment has already returned done = True. You "
    #                 "should always call 'reset()' once you receive 'done = "
    #                 "True' -- any further steps are undefined behavior."
    #             )
    #         self.steps_beyond_done += 1
    #         reward = 0.0
    #
    #     return self.obs, reward, done, {}
    #
    # def reset(self):
    #     # pos0 = self.np_random.choice([XMIN+0.020, XMIN+0.040, XMAX-0.040, XMAX-0.020])
    #     pos0 = self.np_random.uniform(XMIN+0.020, XMAX-0.020)
    #     self.system.reset(np.array([pos0, 0.0]))
    #     self.theta = 0.0
    #     self.pos_est = self.system.observe()
    #     self.vel_est = 0.0
    #     self.obs = np.array([self.pos_est, self.vel_est])
    #     self.t = 0
    #     self.setpoint = self.reference.setpoint(self.t)
    #     self.saturated = False
    #     self.steps_beyond_done = None
    #     return self.obs
    #
    # def render(self, mode='human'):
    #     screen_width = 600
    #     screen_height = 200
    #
    #     world_width = (XMAX-XMIN) * 1.25
    #     scale = screen_width/world_width
    #     axlex, axley = 0.9*screen_width, 0.3*screen_height
    #     beamlength = scale * (XMAX-XMIN)
    #     beamthick = 20
    #     ballrad = 40
    #     tgtballrad = 44
    #     axlerad = beamthick
    #
    #     if self.viewer is None:
    #         from gym.envs.classic_control import rendering
    #         self.viewer = rendering.Viewer(screen_width, screen_height)
    #
    #         # Background
    #         color = tuple(val/255 for val in Monokai.k)
    #         l, r, t, b = 0, screen_width, screen_height, 0
    #         bkgd = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
    #         bkgd.set_color(*color)
    #         self.viewer.add_geom(bkgd)
    #
    #         self.axletrans = rendering.Transform()
    #         self.axletrans.set_translation(axlex, axley)
    #
    #         # Add the beam
    #         l, r, t, b = -beamlength/2, beamlength/2, beamthick/2, -beamthick/2
    #         self.beam = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
    #         color = tuple(val/255 for val in Monokai.b)
    #         self.beam.set_color(*color)
    #         self.beamtrans = rendering.Transform()
    #         self.beam.add_attr(self.beamtrans)
    #         self.beam.add_attr(self.axletrans)
    #         self.viewer.add_geom(self.beam)
    #
    #         # Add the axle
    #         self.axle = rendering.make_circle(radius=axlerad)
    #         self.axle.add_attr(self.axletrans)
    #         color = tuple(val/255 for val in Monokai.s)
    #         self.axle.set_color(*color)
    #         self.viewer.add_geom(self.axle)
    #
    #         # Add the setpoint marker
    #         self.tgtball = rendering.make_circle(radius=tgtballrad, filled=False)
    #         self.tgtball.set_linewidth(2)
    #         self.tgtballtrans = rendering.Transform()
    #         self.tgtball.add_attr(self.tgtballtrans)
    #         self.tgtball.add_attr(self.axletrans)
    #         color = tuple(val/255 for val in Monokai.y)
    #         self.tgtball.set_color(*color)
    #         self.viewer.add_geom(self.tgtball)
    #
    #         # Add the ball
    #         self.ball = rendering.make_circle(radius=ballrad, filled=True)
    #         self.balltrans = rendering.Transform()
    #         self.ball.add_attr(self.balltrans)
    #         self.ball.add_attr(self.axletrans)
    #         color = tuple(val/255 for val in Monokai.w)
    #         self.ball.set_color(*color)
    #         self.viewer.add_geom(self.ball)
    #
    #     if self.obs is None:
    #         return None
    #
    #     # Dynamic updates
    #     x = self.obs[0]
    #     beamx = -math.cos(self.theta)*0.5*beamlength
    #     beamy = -math.sin(self.theta)*0.5*beamlength
    #
    #     ballx = -math.cos(self.theta)*((XMAX-XMIN)-(x-XMIN))*scale
    #     bally = -math.sin(self.theta)*((XMAX-XMIN)-(x-XMIN))*scale + beamthick/2.0 + ballrad
    #
    #     self.balltrans.set_translation(ballx, bally)
    #     self.beamtrans.set_rotation(self.theta)
    #     self.beamtrans.set_translation(beamx, beamy)
    #
    #     x_tgt = self.setpoint
    #     tgtx = -math.cos(self.theta)*((XMAX-XMIN)-(x_tgt-XMIN))*scale
    #     tgty = -math.sin(self.theta)*((XMAX-XMIN)-(x_tgt-XMIN))*scale + beamthick/2.0 + ballrad
    #     self.tgtballtrans.set_translation(tgtx, tgty)
    #
    #     return self.viewer.render(return_rgb_array=mode == 'rgb_array')
    #
    # def close(self):
    #     if self.viewer:
    #         self.viewer.close()
    #         self.viewer = None
    #     self.system.shutdown()

from scipy.special import expit
class Controller:
    def __init__(self):
        self.myobs = np.zeros(2)

    # def policy(self, obs, param):
    #     # Linear observation policy
    #     return np.dot(param, obs)

    def policy(self, obs, param):
        # Linear observation policy w/ observation smoothing
        p = expit(param[2])
        self.myobs = (1-p)*obs + p*self.myobs
        return np.dot(param[0:2], self.myobs)

    def reset(self, obs=None):
        if obs is None:
            obs = np.zeros_like(self.myobs)
        self.myobs = np.copy(obs)
        return self.myobs


if __name__ == '__main__':
    from time import time

    # env = BallBeamEnv(hardware=False)
    env = BallBeamEnv(hardware=True, max_episode_length=400)
    # env.seed(1)

    # from controller import LQGController
    # controller = LQGController()

    controller = Controller()

    action = 0
    time_elapsed = 0
    time_last = time()
    rew_tot = 0
    observation = np.zeros(2)
    # observation = env.reset()
    # observation = env.reset()
    # observation = env.reset()
    # observation = env.reset()
    # myobs = np.copy(observation)
    # controller.reset(myobs)
    for i in range(400):
        time_now = time()
        env.render()

        # # LQG control
        # control_kwargs = dict(saturated=env.saturated)
        # controller.update(observation[0], env.setpoint, env.t, **control_kwargs)
        # action = controller.u

        # # Linear observation feedback
        # weights = np.array([0.5, 0.3])
        # action = np.dot(weights, observation)

        # Linear observation feedback w/ observation smoothing
        # myobs = 0.8*observation + 0.2*myobs
        # weights = np.array([0.5, 0.3])
        # action = np.dot(weights, myobs)

        param2 = np.array([0.6, 0.4, 1.0])
        action = controller.policy(observation, param2)

        # # Random actions
        # action = env.action_space.sample()[0]

        # # Zero action
        # action = 0

        observation, rew, done, info = env.step(action)
        rew_tot += rew

        spacer = '    '
        print(i+1, end='    ')
        print('%12.6f' % observation[0], end=spacer)
        print('%12.6f' % observation[1], end=spacer)
        print('%12.6f' % rew, end=spacer)
        print('%12.6f' % rew_tot, end=spacer)
        print('%12.6f' % (time_now - time_last), end=spacer)
        print('')

        time_last = time_now
        if done:
            break
    env.close()


    # from time import time
    #
    # env = BallBeamDiscreteEnv()
    # env.seed(1)
    # observation = env.reset()
    # action = 0
    # time_elapsed = 0
    # time_last = time()
    # rew_tot = 0
    # for i in range(400):
    #     time_now = time()
    #     env.render()
    #     observation, rew, done, info = env.step(action)
    #     pos_est, vel_est = observation
    #
    #     # # Random actions
    #     # action = env.action_space.sample()
    #
    #     # Handcrafted stochastic policy
    #     p1 = int(pos_est > env.setpoint)  # policy associated with position
    #     p2 = (1+np.tanh(8*vel_est))/2  # policy associated with velocity
    #     q = 0.6*p1 + 0.4*p2  # blend policies 1 and 2 in convex combination
    #     p = np.array([1-q, q])  # full probability vector
    #     action = np.random.choice([0, 1], p=p)
    #
    #     rew_tot += rew
    #
    #     spacer = '    '
    #     print(i+1, end='    ')
    #     print('%12.6f' % observation[0], end=spacer)
    #     print('%12.6f' % observation[1], end=spacer)
    #     print('%12.6f' % rew, end=spacer)
    #     print('%12.6f' % rew_tot, end=spacer)
    #     print('%12.6f' % (time_now - time_last), end=spacer)
    #     print('')
    #     time_last = time_now
    #     if done:
    #         break
    # env.close()
