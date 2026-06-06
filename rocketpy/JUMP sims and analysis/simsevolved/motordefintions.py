from math import pi, sqrt
from pathlib import Path

from rocketpy import SolidMotor


MOTOR_DIR = Path(__file__).resolve().parent


def resolve_motor_file(thrust_source):
    """Resolve motor files relative to this definitions file."""

    path = Path(thrust_source)
    if path.is_absolute():
        return path
    return MOTOR_DIR / path


def propellant_mass_from_eng(thrust_source):
    """Read propellant mass in kg from the first header line of a RASP .eng file."""

    motor_file = resolve_motor_file(thrust_source)
    with motor_file.open("r", encoding="utf-8") as eng_file:
        for line in eng_file:
            stripped = line.strip()
            if not stripped or stripped.startswith(";"):
                continue

            parts = stripped.split()
            if len(parts) < 5:
                raise ValueError(f"Invalid .eng header in {motor_file}: {stripped}")
            return float(parts[4])

    raise ValueError(f"No .eng header found in {motor_file}")


def grain_initial_inner_radius_from_eng(
    thrust_source,
    grain_number,
    grain_density,
    grain_outer_radius,
    grain_initial_height,
):
    """Calculate grain inner radius that matches .eng propellant mass."""

    propellant_mass = propellant_mass_from_eng(thrust_source)
    full_grain_area = grain_outer_radius**2
    propellant_area = propellant_mass / (
        grain_number * grain_density * pi * grain_initial_height
    )
    inner_radius_squared = full_grain_area - propellant_area

    if inner_radius_squared <= 0:
        raise ValueError(
            "The .eng propellant mass is too large for the provided grain "
            f"geometry in {thrust_source}."
        )

    inner_radius = sqrt(inner_radius_squared)
    if inner_radius >= grain_outer_radius:
        raise ValueError(
            "The calculated grain inner radius is larger than the outer radius "
            f"for {thrust_source}."
        )

    return inner_radius


BOOSTER_THRUST_SOURCE = "./Chase_M2500.eng"
BOOSTER_GRAIN_NUMBER = 1
BOOSTER_GRAIN_DENSITY = 1815
BOOSTER_GRAIN_OUTER_RADIUS = 27 / 1000
BOOSTER_GRAIN_INITIAL_HEIGHT = 1193 / 1000

SUSTAINER_THRUST_SOURCE = "./L2523.eng"
SUSTAINER_GRAIN_NUMBER = 1
SUSTAINER_GRAIN_DENSITY = 1815
SUSTAINER_GRAIN_OUTER_RADIUS = 23.8 / 1000
SUSTAINER_GRAIN_INITIAL_HEIGHT = 787 / 1000


BoosterMotor = SolidMotor(
thrust_source=str(resolve_motor_file(BOOSTER_THRUST_SOURCE)),
dry_mass=0, #kg
dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
nozzle_radius=25 / 1000, #m
grain_number=BOOSTER_GRAIN_NUMBER, #m
grain_density=BOOSTER_GRAIN_DENSITY, #kg/m^3
grain_outer_radius=BOOSTER_GRAIN_OUTER_RADIUS, #m
grain_initial_inner_radius=grain_initial_inner_radius_from_eng(
    BOOSTER_THRUST_SOURCE,
    BOOSTER_GRAIN_NUMBER,
    BOOSTER_GRAIN_DENSITY,
    BOOSTER_GRAIN_OUTER_RADIUS,
    BOOSTER_GRAIN_INITIAL_HEIGHT,
), #m
grain_initial_height=BOOSTER_GRAIN_INITIAL_HEIGHT, #m
grain_separation=1 / 1000, #m
grains_center_of_mass_position=0.45, #m
center_of_dry_mass_position=0.317, #m
nozzle_position=-0.19, #m
#burn_time=2.1, #s
throat_radius=16.5 / 1000,#m
coordinate_system_orientation="nozzle_to_combustion_chamber"
)
# BoosterMotor.draw()
#BoosterMotor.info()

SustainerMotor = SolidMotor(
thrust_source=str(resolve_motor_file(SUSTAINER_THRUST_SOURCE)),
dry_mass=0, #kg
dry_inertia=(0.125, 0.125, 0.002), #moi ,kg/m^2
nozzle_radius=25 / 1000, #m
grain_number=SUSTAINER_GRAIN_NUMBER, #m
grain_density=SUSTAINER_GRAIN_DENSITY, #kg/m^3
grain_outer_radius=SUSTAINER_GRAIN_OUTER_RADIUS, #m
grain_initial_inner_radius=grain_initial_inner_radius_from_eng(
    SUSTAINER_THRUST_SOURCE,
    SUSTAINER_GRAIN_NUMBER,
    SUSTAINER_GRAIN_DENSITY,
    SUSTAINER_GRAIN_OUTER_RADIUS,
    SUSTAINER_GRAIN_INITIAL_HEIGHT,
), #m
grain_initial_height=SUSTAINER_GRAIN_INITIAL_HEIGHT, #m
grain_separation=1 / 1000, #m
grains_center_of_mass_position=0.403, #m
center_of_dry_mass_position=0.317, #m
nozzle_position=-0.04, #m
#burn_time=3.5, #s
throat_radius=16.5 / 1000,#m
coordinate_system_orientation="nozzle_to_combustion_chamber",

)
#SustainerMotor.draw()
#SustainerMotor.info()
