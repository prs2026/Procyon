import multiprocessing
import pickle

import numpy as np
from rocketpy import Environment, Flight

from motordefintions import BoosterMotor
from openmeteo_environment import create_openmeteo_environment
from rocketdefintions import JUMPStack, JUMPSustainer, JUMPSustainerNOMOTOR


# Launch site: FAR, 35.3466 N, 117.809 W, 2000 ft MSL
site_lat = 35.3466
site_lon = -117.809
site_alt = 2000 / 3.281

railinclination = [89, 5]
railheading = [90, 180]

BLUE_RAVEN_LOOKAHEAD = 3
STAGING_VELOCITY_LIMIT = 700 / 3.281
STAGING_ANGLE_LIMIT = 9
ABORT_VELOCITY_LIMIT = 300 / 3.281
ABORT_ANGLE_LIMIT = 15

env = None


def create_sim_environment():
    """Fetch Open-Meteo once and build a RocketPy environment."""

    result = create_openmeteo_environment(
        latitude=site_lat,
        longitude=site_lon,
        date="2026-06-07",
        time="12:00",
        timezone_name="US/Pacific",
        elevation_m=site_alt,
    )
    return result.environment, {
        "pressure": result.pressure,
        "temperature": result.temperature,
        "wind_u": result.wind_u,
        "wind_v": result.wind_v,
        "latitude": site_lat,
        "longitude": site_lon,
        "elevation": result.environment.elevation,
        "timezone": result.environment.timezone,
        "date": result.environment.date,
        "max_expected_height": result.environment.max_expected_height,
    }


def environment_from_profile(profile):
    """Rebuild RocketPy Environment in each worker without another API call."""

    worker_env = Environment(
        latitude=profile["latitude"],
        longitude=profile["longitude"],
        elevation=profile["elevation"],
        timezone=profile["timezone"],
        date=profile["date"],
        max_expected_height=profile["max_expected_height"],
    )
    worker_env.set_atmospheric_model(
        type="custom_atmosphere",
        pressure=profile["pressure"],
        temperature=profile["temperature"],
        wind_u=profile["wind_u"],
        wind_v=profile["wind_v"],
    )
    return worker_env


def init_worker(environment_profile):
    global env
    env = environment_from_profile(environment_profile)


def get_environment():
    """Fallback for single-process/debug calls to runfullstacksim."""

    global env
    if env is None:
        env, _ = create_sim_environment()
    return env


def quaternion_to_angle_from_vertical(e0, e1, e2, e3):
    norm = np.sqrt(e0**2 + e1**2 + e2**2 + e3**2)
    e0, e1, e2, e3 = e0 / norm, e1 / norm, e2 / norm, e3 / norm
    vz = e0**2 - e1**2 - e2**2 + e3**2
    return np.degrees(np.arccos(np.clip(vz, -1, 1)))


def solution_angle_from_vertical(solution_row):
    return quaternion_to_angle_from_vertical(
        solution_row[7],
        solution_row[8],
        solution_row[9],
        solution_row[10],
    )


def future_tilt_angle(current_angle, previous_angle, current_time, previous_time):
    if previous_angle is None or previous_time is None:
        return current_angle

    dt = current_time - previous_time
    if dt <= 0:
        return current_angle

    tilt_rate = (current_angle - previous_angle) / dt
    return max(0, current_angle + tilt_rate * BLUE_RAVEN_LOOKAHEAD)


def runfullstacksim(val):
    sim_env = get_environment()
    rng = np.random.default_rng()

    StackFlight2 = Flight(
        rocket=JUMPStack,
        environment=sim_env,
        rail_length=6,
        inclination=rng.normal(railinclination[0], railinclination[1]),
        heading=rng.normal(railheading[0], railheading[1]),
        max_time=BoosterMotor.burn_out_time,
        rtol=1e-4,
        atol=1e-6,
        time_overshoot=True,
    )

    boosterburnoutangle = solution_angle_from_vertical(StackFlight2.solution[-1])
    if boosterburnoutangle > 14:
        return StackFlight2.solution

    maxstagingdelay = 15
    SustainerNOMOTORFlight2 = Flight(
        rocket=JUMPSustainerNOMOTOR,
        environment=sim_env,
        initial_solution=StackFlight2,
        rail_length=0.01,
        inclination=StackFlight2.attitude_angle(BoosterMotor.burn_out_time - 0.01),
        heading=StackFlight2.path_angle(BoosterMotor.burn_out_time - 0.01),
        max_time=maxstagingdelay + BoosterMotor.burn_out_time,
        rtol=1e-4,
        atol=1e-6,
        time_overshoot=True,
    )

    stagingindex = len(SustainerNOMOTORFlight2.solution) - 1
    stagingtime = SustainerNOMOTORFlight2.solution[stagingindex][0]
    previousangle = None
    previoustime = None

    for index, point in enumerate(SustainerNOMOTORFlight2.solution):
        currentangle = solution_angle_from_vertical(point)
        futureangle = future_tilt_angle(
            currentangle,
            previousangle,
            point[0],
            previoustime,
        )

        # Abort conditions must be checked before staging conditions.
        if point[6] < ABORT_VELOCITY_LIMIT or currentangle > ABORT_ANGLE_LIMIT:
            return SustainerNOMOTORFlight2.solution

        if (
            point[6] < STAGING_VELOCITY_LIMIT
            or currentangle > STAGING_ANGLE_LIMIT
            or futureangle > STAGING_ANGLE_LIMIT
        ):
            stagingindex = index
            stagingtime = point[0]
            break

        previousangle = currentangle
        previoustime = point[0]

    sustainerstartcondition = SustainerNOMOTORFlight2.solution[stagingindex][:]
    sustainerstartcondition[0] = 0

    SustainerFlight2 = Flight(
        rocket=JUMPSustainer,
        environment=sim_env,
        initial_solution=sustainerstartcondition,
        rail_length=0.01,
        inclination=SustainerNOMOTORFlight2.attitude_angle(stagingtime - 0.01),
        heading=SustainerNOMOTORFlight2.path_angle(stagingtime - 0.01),
        rtol=1e-4,
        atol=1e-6,
        time_overshoot=True,
    )

    # stagingtime is already absolute time since launch.
    for entry in SustainerFlight2.solution:
        entry[0] += stagingtime

    return SustainerFlight2.solution


def main():
    numsims = int(input("How many runs? "))
    numworkers = int(input("How many workers? "))

    print("Fetching atmosphere once...")
    _, environment_profile = create_sim_environment()

    print("Parachutes on rocket:")
    for chute in JUMPSustainer.parachutes:
        print(chute)

    numbers = list(range(numsims))
    chunksize = max(1, len(numbers) // (numworkers * 4)) if numbers else 1

    with multiprocessing.Pool(
        processes=numworkers,
        initializer=init_worker,
        initargs=(environment_profile,),
    ) as pool:
        results = pool.map(runfullstacksim, numbers, chunksize=chunksize)

    print("done with sims")

    with open("flight_data.pkl", "wb") as f:
        pickle.dump(results, f)

    print("results saved to pkl file")


if __name__ == "__main__":
    multiprocessing.freeze_support()
    main()
