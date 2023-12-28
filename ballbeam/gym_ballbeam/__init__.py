import logging

from gym.envs.registration import register

logger = logging.getLogger(__name__)

register(
    id="BallBeamContinuous-v0",
    entry_point="gym_ballbeam.envs:BallBeamContinuousEnv",
)

register(
    id="BallBeamDiscrete-v0",
    entry_point="gym_ballbeam.envs:BallBeamDiscreteEnv",
)
