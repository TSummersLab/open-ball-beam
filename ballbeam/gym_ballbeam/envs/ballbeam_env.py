"""Open Ball & Beam gym environment."""

from __future__ import annotations

import math
from typing import Any, ClassVar

import gym  # type: ignore[import]
import numpy as np
from gym import logger, spaces  # type: ignore[import]
from gym.envs.classic_control import rendering  # type: ignore[import]
from gym.utils import seeding  # type: ignore[import]

from ballbeam.common.colors import Monokai
from ballbeam.common.cost import Cost
from ballbeam.common.hardware import HardwareSystem
from ballbeam.common.interface import REFERENCE_CLASS_MAP, instantiate_object_by_class_name
from ballbeam.common.simulator import XMAX, XMIN, YMAX, YMIN, SimulatorSystem
from ballbeam.configurators.configs import CONFIG


class BallBeamEnv(gym.Env):
    """Open Ball & Beam gym base environment."""

    metadata: ClassVar[dict[str, Any]] = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}

    def __init__(
        self,
        max_episode_length: int = 200,
        reference_type: str = "Constant",
        *,
        use_hardware: bool = False,
        seed: int | None = None,
    ) -> None:
        """Initialize."""
        self.seed(seed)
        self.viewer: rendering.Viewer | None = None
        self.umin = CONFIG.hardware.BEAM.ANGLE.MIN * CONFIG.constants.DEG2RAD
        self.umax = CONFIG.hardware.BEAM.ANGLE.MAX * CONFIG.constants.DEG2RAD
        self.action_space = None
        obs_lo, obs_hi = np.array(YMIN, dtype=np.float32), np.array(YMAX, dtype=np.float32)
        self.observation_space = spaces.Box(obs_lo, obs_hi, dtype=np.float32)
        self.max_episode_length = max_episode_length
        self.system: HardwareSystem | SimulatorSystem = (
            HardwareSystem() if use_hardware else SimulatorSystem(process_noise_scale=1.0, sensor_noise_scale=1.0)
        )
        self.reference = instantiate_object_by_class_name(reference_type, REFERENCE_CLASS_MAP)
        self.cost = Cost()

        self.setpoint: float = 0.0
        self.steps_beyond_done: int | None = None

        self.reset()

    def seed(self, seed: int | None = None) -> list[int | None]:
        """Seed the environment."""
        self.np_random, _seed = seeding.np_random(seed)
        # TODO(bgravell): NPY002 Replace legacy `np.random.seed` call with `np.random.Generator`  # noqa: TD003, FIX002
        np.random.seed(seed)  # noqa: NPY002
        return [seed]

    def action2theta(self, action: Any) -> float:
        """Convert action to beam angle theta."""
        raise NotImplementedError

    def step(self, action: Any) -> tuple[float, float, bool, dict[str, Any]]:
        """Take a single step in the environment.

        This function mirrors ballbeam.common.interface.update()
        """
        theta = self.action2theta(action)
        theta = np.clip(theta, self.umin, self.umax)
        self.theta = theta

        # Get a new observation from the system
        self.observation = self.system.observe()

        # Get the current setpoint from the reference
        self.setpoint = self.reference.setpoint(self.t)

        # Get the current cost of the observation and theta
        cost_dict = self.cost.take(self.observation, theta)
        reward = -cost_dict["c"]

        # Evolve the system forward by one step
        self.system.process(theta)

        # Set the done status
        done = bool(self.t + 1 >= self.max_episode_length)

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

        info: dict[str, Any] = {}

        return self.observation, reward, done, info

    def reset(self, pos0: float | None = None) -> float:
        """Reset the environment."""
        if pos0 is None:
            pos0 = self.np_random.uniform(XMIN + 0.020, XMAX - 0.020)
        self.system.reset(np.array([pos0, 0.0]))
        self.theta = 0.0
        self.t = 0
        self.setpoint = self.reference.setpoint(self.t)
        self.observation = pos0
        self.saturated = False
        self.steps_beyond_done = None
        return self.observation

    def render(self, mode: str = "human") -> Any:  # noqa: PLR0915
        """Render the environment."""
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
            self.viewer = rendering.Viewer(screen_width, screen_height)

            # Background
            color = tuple(val / 255 for val in Monokai.k)
            left, right, top, bottom = 0.0, float(screen_width), float(screen_height), 0.0
            bkgd = rendering.FilledPolygon([(left, bottom), (left, top), (right, top), (right, bottom)])
            bkgd.set_color(*color)
            self.viewer.add_geom(bkgd)

            self.axletrans = rendering.Transform()
            self.axletrans.set_translation(axlex, axley)

            # Add the beam
            left, right, top, bottom = -beamlength / 2, beamlength / 2, beamthick / 2, -beamthick / 2
            self.beam = rendering.FilledPolygon([(left, bottom), (left, top), (right, top), (right, bottom)])
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
        x = self.observation
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
        """Close the environment."""
        if self.viewer:
            self.viewer.close()
            self.viewer = None
        self.system.shutdown()


class BallBeamContinuousEnv(BallBeamEnv):
    """Open Ball & Beam gym environment with a continuous action space."""

    def __init__(self, **kwargs: Any) -> None:
        """Initialize."""
        super().__init__(**kwargs)
        self.action_space = spaces.Box(self.umin, self.umax, shape=(1,), dtype=np.float32)

    def action2theta(self, action: float) -> float:
        """Convert action to beam angle theta."""
        return action


class BallBeamDiscreteEnv(BallBeamEnv):
    """Open Ball & Beam gym environment with a discrete action space.

    The discrete action space accepts the following values:
    0: Negative beam angle.
    1: Zero beam angle.
    2: Positive beam amgle.
    """

    def __init__(self, **kwargs: Any) -> None:
        """Initialize."""
        super().__init__(**kwargs)
        self.action_space = spaces.Discrete(3)
        self.action_map = {
            0: 3 * CONFIG.constants.DEG2RAD,
            1: 0.0,
            2: -3 * CONFIG.constants.DEG2RAD,
        }

    def action2theta(self, action: int) -> float:
        """Convert action to beam angle theta."""
        return self.action_map[action]


if __name__ == "__main__":
    from time import time

    from ballbeam.common.interface import CONTROLLER_CLASS_MAP

    use_discrete_env = False
    use_hardware = False
    max_episode_length = 400
    seed = None

    # Instantiate the Gym environment
    Env = BallBeamDiscreteEnv if use_discrete_env else BallBeamContinuousEnv
    env = Env(
        reference_type="SlowSine",
        use_hardware=use_hardware,
        max_episode_length=max_episode_length,
    )
    env.seed(seed)
    # TOD(bgravell): harmonize random seeding of the simulator in simulator.py and Gym

    # Instantiate controller
    controller = instantiate_object_by_class_name(CONFIG.interface.controller_type, CONTROLLER_CLASS_MAP)

    # Initialize
    action = 0
    time_elapsed = 0
    time_last = time()
    reward_tot = 0.0
    observation = env.reset()

    spacer = "    "
    print(
        f'{"time":12s}{spacer}{"observation":12s}{spacer}{"setpoint":12s}{spacer}{"reward":12s}{spacer}{"reward_tot":12s}{spacer}{"clock_dt":12s}{spacer}',
    )

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
            # Only act if the requested cont_action is large
            if np.abs(cont_action) > 0.02:  # noqa: SIM108, PLR2004
                action = int(np.sign(cont_action) + 1)
            else:
                action = 1
        else:
            action = cont_action

        observation, reward, done, info = env.step(action)
        reward_tot += reward

        clock_dt = time_now - time_last

        print(
            f"{i+1:12d}{spacer}{observation:12.3f}{spacer}{env.setpoint:12.3f}{spacer}{reward:12.3f}{spacer}{reward_tot:12.3f}{spacer}{clock_dt:12.3f}",
        )

        time_last = time_now
        if done:
            break
    env.close()
