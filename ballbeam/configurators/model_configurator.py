"""Configurator for simulation model."""

from ballbeam.configurators.configurators import Configurator


def make_model_configurator(constants_configurator: Configurator) -> Configurator:
    """Make a configurator for the model."""
    name = "model"
    description = "Model constants"

    constants_config = constants_configurator.data_obj

    # Physical constants

    # meters/second/second
    GRAVITY = constants_config.STD_GRAVITY_ACCEL  # type: ignore[attr-defined]

    # Ball
    BALL_RADIUS = 0.01905  # meters
    BALL_MASS = 0.039  # kilograms
    # Scaling constant of mass resulting from the ball's rotational inertia.
    # This value is for a solid sphere since J = (2/5)*m*R**2
    MASS_SCALE = 1.0 + (2.0 / 5.0)

    # Friction damping
    DAMP = 0.01  # 1/second

    # Motor characteristics

    # rad/sec
    MOTOR_SPEED = (60 / 0.16) * constants_config.DEG2RAD  # type: ignore[attr-defined]
    # Transition rate controls how fast np.tanh() switches from from -1 to +1
    # i.e. higher transition rate gives a better approximation of np.sign() by np.tanh(), but less smooth
    TRANSITION_RATE = 100.0

    # Derived quantities
    GRAVITY_SCALED = GRAVITY / MASS_SCALE
    DAMP_SCALED = DAMP / (BALL_MASS * MASS_SCALE)

    data = {
        "GRAVITY": GRAVITY,
        "GRAVITY_SCALED": GRAVITY_SCALED,
        "BALL_RADIUS": BALL_RADIUS,
        "BALL_MASS": BALL_MASS,
        "MASS_SCALE": MASS_SCALE,
        "DAMP": DAMP,
        "DAMP_SCALED": DAMP_SCALED,
        "MOTOR_SPEED": MOTOR_SPEED,
        "TRANSITION_RATE": TRANSITION_RATE,
    }
    return Configurator(name, data, description)
