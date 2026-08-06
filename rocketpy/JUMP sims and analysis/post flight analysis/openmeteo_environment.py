"""Build a RocketPy Environment from Open-Meteo pressure-level data.

This module fetches only the weather fields needed by RocketPy's
``custom_atmosphere`` model:

- pressure as a function of altitude
- temperature as a function of altitude
- east wind component as a function of altitude
- north wind component as a function of altitude

Open-Meteo pressure-level data usually ends near 26 km for the GFS endpoint.
For high-altitude flights, the pressure and temperature profiles are extended
above the model top with an ISA-shaped profile matched at the splice point.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from datetime import datetime
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import urlopen

import numpy as np
from rocketpy import Environment


FORECAST_GFS_URL = "https://api.open-meteo.com/v1/gfs"
HISTORICAL_FORECAST_URL = "https://historical-forecast-api.open-meteo.com/v1/forecast"

GFS_PRESSURE_LEVELS_HPA = (
    1000,
    975,
    950,
    925,
    900,
    875,
    850,
    825,
    800,
    775,
    750,
    725,
    700,
    675,
    650,
    625,
    600,
    575,
    550,
    525,
    500,
    475,
    450,
    425,
    400,
    375,
    350,
    325,
    300,
    275,
    250,
    225,
    200,
    175,
    150,
    125,
    100,
    70,
    50,
    40,
    30,
    20,
    15,
    10,
)

HISTORICAL_PRESSURE_LEVELS_HPA = (
    1000,
    975,
    950,
    925,
    900,
    850,
    800,
    700,
    600,
    500,
    400,
    300,
    250,
    200,
    150,
    100,
    70,
    50,
    30,
)


@dataclass(frozen=True)
class OpenMeteoEnvironmentResult:
    """RocketPy environment plus the profile arrays used to create it."""

    environment: Environment
    pressure: np.ndarray
    temperature: np.ndarray
    wind_u: np.ndarray
    wind_v: np.ndarray
    model_top_height_m: float
    request_url: str
    response: dict[str, Any]


def requested_hour(date: str, time: str) -> datetime:
    """Parse a local Open-Meteo hour string."""

    return datetime.fromisoformat(f"{date}T{time}")


def pressure_level_variables(levels_hpa: tuple[int, ...]) -> list[str]:
    """Return the pressure-level variable names needed for the query."""

    fields = ("temperature", "wind_speed", "wind_direction", "geopotential_height")
    return [f"{field}_{level}hPa" for level in levels_hpa for field in fields]


def hourly_variables(levels_hpa: tuple[int, ...]) -> list[str]:
    """Return the complete minimal hourly variable set for RocketPy."""

    return [
        "surface_pressure",
        "temperature_2m",
        "wind_speed_10m",
        "wind_direction_10m",
        *pressure_level_variables(levels_hpa),
    ]


def choose_endpoint(mode: str, hour: datetime) -> str:
    """Choose the Open-Meteo endpoint for forecast, historical, or auto mode."""

    if mode == "forecast":
        return FORECAST_GFS_URL
    if mode == "historical":
        return HISTORICAL_FORECAST_URL

    return HISTORICAL_FORECAST_URL if hour < datetime.now() else FORECAST_GFS_URL


def default_levels_for_endpoint(base_url: str) -> tuple[int, ...]:
    """Return the pressure levels supported by the selected endpoint."""

    if "historical-forecast-api" in base_url:
        return HISTORICAL_PRESSURE_LEVELS_HPA
    return GFS_PRESSURE_LEVELS_HPA


def fetch_openmeteo_hour(
    latitude: float,
    longitude: float,
    date: str,
    time: str,
    *,
    timezone_name: str = "UTC",
    elevation_m: float | None = None,
    model: str | None = "gfs_seamless",
    endpoint: str = "auto",
    timeout_s: float = 30,
    levels_hpa: tuple[int, ...] | None = None,
) -> tuple[dict[str, Any], str, tuple[int, ...]]:
    """Fetch one hourly Open-Meteo profile for a location and local time."""

    hour = requested_hour(date, time)
    base_url = choose_endpoint(endpoint, hour)
    levels_hpa = levels_hpa or default_levels_for_endpoint(base_url)

    params: dict[str, Any] = {
        "latitude": latitude,
        "longitude": longitude,
        "timezone": timezone_name,
        "start_hour": hour.strftime("%Y-%m-%dT%H:%M"),
        "end_hour": hour.strftime("%Y-%m-%dT%H:%M"),
        "timeformat": "iso8601",
        "temperature_unit": "celsius",
        "wind_speed_unit": "ms",
        "hourly": ",".join(hourly_variables(levels_hpa)),
    }
    if elevation_m is not None:
        params["elevation"] = elevation_m
    if model:
        params["models"] = model

    request_url = f"{base_url}?{urlencode(params)}"

    try:
        with urlopen(request_url, timeout=timeout_s) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except HTTPError as exc:
        body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"Open-Meteo HTTP {exc.code}: {body}") from exc
    except URLError as exc:
        raise RuntimeError(f"Open-Meteo request failed: {exc.reason}") from exc

    if payload.get("error"):
        raise RuntimeError(f"Open-Meteo error: {payload.get('reason', payload)}")

    hourly = payload.get("hourly", {})
    if not hourly.get("time"):
        raise RuntimeError("Open-Meteo returned no hourly data for the requested hour.")

    return payload, request_url, levels_hpa


def first_hour_value(hourly: dict[str, Any], key: str) -> float | None:
    """Read the first hourly value for a key, preserving missing values."""

    values = hourly.get(key)
    if not values:
        return None
    value = values[0]
    return None if value is None else float(value)


def wind_components(speed_m_s: float, direction_deg: float) -> tuple[float, float]:
    """Convert meteorological direction into east/north wind components."""

    theta = math.radians(direction_deg)
    wind_u = -speed_m_s * math.sin(theta)
    wind_v = -speed_m_s * math.cos(theta)
    return wind_u, wind_v


def clean_profile(rows: list[tuple[float, float]]) -> np.ndarray:
    """Sort, filter, and de-duplicate a height/value profile."""

    valid_rows = [
        (float(height), float(value))
        for height, value in rows
        if np.isfinite(height) and np.isfinite(value)
    ]
    if not valid_rows:
        raise RuntimeError("No valid rows found while building an atmosphere profile.")

    deduped: dict[float, float] = {}
    for height, value in sorted(valid_rows):
        deduped[round(height, 3)] = value

    return np.array([[height, value] for height, value in deduped.items()], dtype=float)


def clean_pressure_profile(rows: list[tuple[float, float]]) -> np.ndarray:
    """Clean a pressure profile and keep it invertible for RocketPy."""

    cleaned = clean_profile(rows)
    monotonic_rows: list[tuple[float, float]] = []
    for height, pressure in cleaned:
        if not monotonic_rows or pressure < monotonic_rows[-1][1]:
            monotonic_rows.append((float(height), float(pressure)))

    if len(monotonic_rows) < 2:
        raise RuntimeError("Pressure profile has fewer than two monotonic points.")

    return np.array(monotonic_rows, dtype=float)


def append_profile_rows(profile: np.ndarray, rows: list[tuple[float, float]]) -> np.ndarray:
    """Append rows to a profile and re-clean it."""

    combined = [(float(height), float(value)) for height, value in profile]
    combined.extend(rows)
    return clean_profile(combined)


def append_pressure_rows(profile: np.ndarray, rows: list[tuple[float, float]]) -> np.ndarray:
    """Append rows to a pressure profile and enforce monotonic pressure."""

    combined = [(float(height), float(value)) for height, value in profile]
    combined.extend(rows)
    return clean_pressure_profile(combined)



def extend_pressure_temperature_with_isa(
    pressure: np.ndarray,
    temperature: np.ndarray,
    *,
    latitude: float,
    longitude: float,
    elevation_m: float,
    date_tuple: tuple[int, int, int, int],
    timezone_name: str,
    max_expected_height_m: float,
    extension_points: int = 80,
) -> tuple[np.ndarray, np.ndarray]:
    """Extend pressure and temperature above Open-Meteo's highest level."""

    top_height = min(float(pressure[-1, 0]), float(temperature[-1, 0]))
    if max_expected_height_m <= top_height:
        return pressure, temperature

    standard_env = Environment(
        latitude=latitude,
        longitude=longitude,
        elevation=elevation_m,
        timezone=timezone_name,
        date=date_tuple,
        max_expected_height=max_expected_height_m,
    )
    standard_env.set_atmospheric_model(type="standard_atmosphere")

    pressure_scale = float(pressure[-1, 1]) / float(standard_env.pressure(top_height))
    temperature_offset = float(temperature[-1, 1]) - float(standard_env.temperature(top_height))

    heights = np.linspace(top_height, max_expected_height_m, extension_points + 1)[1:]
    pressure_extension = [
        (height, float(standard_env.pressure(height)) * pressure_scale)
        for height in heights
    ]
    temperature_extension = [
        (height, float(standard_env.temperature(height)) + temperature_offset)
        for height in heights
    ]

    return (
        append_pressure_rows(pressure, pressure_extension),
        append_profile_rows(temperature, temperature_extension),
    )


def extend_wind_constant(
    wind_profile: np.ndarray,
    *,
    max_expected_height_m: float,
    extension_points: int = 20,
) -> np.ndarray:
    """Hold wind constant above Open-Meteo's model top."""

    top_height = float(wind_profile[-1, 0])
    if max_expected_height_m <= top_height:
        return wind_profile

    top_value = float(wind_profile[-1, 1])
    heights = np.linspace(top_height, max_expected_height_m, extension_points + 1)[1:]
    return append_profile_rows(wind_profile, [(height, top_value) for height in heights])


def build_profiles(
    payload: dict[str, Any],
    *,
    elevation_m: float | None = None,
    levels_hpa: tuple[int, ...] = GFS_PRESSURE_LEVELS_HPA,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Convert Open-Meteo JSON into RocketPy profile arrays."""

    hourly = payload["hourly"]
    launch_elevation = float(elevation_m if elevation_m is not None else payload["elevation"])

    surface_pressure = first_hour_value(hourly, "surface_pressure")
    surface_temperature = first_hour_value(hourly, "temperature_2m")
    surface_wind_speed = first_hour_value(hourly, "wind_speed_10m")
    surface_wind_direction = first_hour_value(hourly, "wind_direction_10m")

    pressure_rows: list[tuple[float, float]] = []
    temperature_rows: list[tuple[float, float]] = []
    wind_u_rows: list[tuple[float, float]] = []
    wind_v_rows: list[tuple[float, float]] = []

    if surface_pressure is not None:
        pressure_rows.append((launch_elevation, surface_pressure * 100.0))
    if surface_temperature is not None:
        temperature_rows.append((launch_elevation + 2.0, surface_temperature + 273.15))
    if surface_wind_speed is not None and surface_wind_direction is not None:
        wind_u, wind_v = wind_components(surface_wind_speed, surface_wind_direction)
        wind_u_rows.append((launch_elevation + 10.0, wind_u))
        wind_v_rows.append((launch_elevation + 10.0, wind_v))

    for level in levels_hpa:
        height = first_hour_value(hourly, f"geopotential_height_{level}hPa")
        temperature = first_hour_value(hourly, f"temperature_{level}hPa")
        wind_speed = first_hour_value(hourly, f"wind_speed_{level}hPa")
        wind_direction = first_hour_value(hourly, f"wind_direction_{level}hPa")

        if height is None:
            continue
        pressure_rows.append((height, level * 100.0))
        if temperature is not None:
            temperature_rows.append((height, temperature + 273.15))
        if wind_speed is not None and wind_direction is not None:
            wind_u, wind_v = wind_components(wind_speed, wind_direction)
            wind_u_rows.append((height, wind_u))
            wind_v_rows.append((height, wind_v))

    return (
        clean_pressure_profile(pressure_rows),
        clean_profile(temperature_rows),
        clean_profile(wind_u_rows),
        clean_profile(wind_v_rows),
        launch_elevation,
    )


def create_openmeteo_environment(
    latitude: float,
    longitude: float,
    date: str,
    time: str,
    *,
    timezone_name: str = "UTC",
    elevation_m: float | None = None,
    model: str | None = "gfs_seamless",
    endpoint: str = "auto",
    max_expected_height_m: float = 80000.0,
    extend_above_model_top: bool = True,
) -> OpenMeteoEnvironmentResult:
    """Fetch Open-Meteo data and return a configured RocketPy Environment."""

    payload, request_url, levels_hpa = fetch_openmeteo_hour(
        latitude,
        longitude,
        date,
        time,
        timezone_name=timezone_name,
        elevation_m=elevation_m,
        model=model,
        endpoint=endpoint,
    )
    pressure, temperature, wind_u, wind_v, launch_elevation = build_profiles(
        payload,
        elevation_m=elevation_m,
        levels_hpa=levels_hpa,
    )
    model_top_height_m = max(
        float(pressure[-1, 0]),
        float(temperature[-1, 0]),
        float(wind_u[-1, 0]),
        float(wind_v[-1, 0]),
    )

    date_tuple = requested_hour(date, time).timetuple()[:4]
    if extend_above_model_top:
        pressure, temperature = extend_pressure_temperature_with_isa(
            pressure,
            temperature,
            latitude=latitude,
            longitude=longitude,
            elevation_m=launch_elevation,
            date_tuple=date_tuple,
            timezone_name=timezone_name,
            max_expected_height_m=max_expected_height_m,
        )
        wind_u = extend_wind_constant(wind_u, max_expected_height_m=max_expected_height_m)
        wind_v = extend_wind_constant(wind_v, max_expected_height_m=max_expected_height_m)

    env = Environment(
        latitude=latitude,
        longitude=longitude,
        elevation=launch_elevation,
        timezone=timezone_name,
        date=date_tuple,
        max_expected_height=max_expected_height_m,
    )
    env.set_atmospheric_model(
        type="custom_atmosphere",
        pressure=pressure,
        temperature=temperature,
        wind_u=wind_u,
        wind_v=wind_v,
    )

    return OpenMeteoEnvironmentResult(
        environment=env,
        pressure=pressure,
        temperature=temperature,
        wind_u=wind_u,
        wind_v=wind_v,
        model_top_height_m=model_top_height_m,
        request_url=request_url,
        response=payload,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a RocketPy Environment from Open-Meteo pressure-level data."
    )
    parser.add_argument("--latitude", type=float, required=True)
    parser.add_argument("--longitude", type=float, required=True)
    parser.add_argument("--date", required=True, help="Local date as YYYY-MM-DD")
    parser.add_argument("--time", required=True, help="Local hour as HH:MM")
    parser.add_argument("--timezone", default="UTC", help="IANA timezone, e.g. US/Pacific")
    parser.add_argument("--elevation-m", type=float, default=None)
    parser.add_argument("--model", default="gfs_seamless")
    parser.add_argument(
        "--endpoint",
        choices=("auto", "forecast", "historical"),
        default="auto",
    )
    parser.add_argument("--max-expected-height-m", type=float, default=80000.0)
    parser.add_argument(
        "--no-isa-extension",
        action="store_true",
        help="Leave Open-Meteo profiles unextended above the model top.",
    )
    parser.add_argument(
        "--info",
        action="store_true",
        help="Print RocketPy environment info after creating the environment.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = create_openmeteo_environment(
        args.latitude,
        args.longitude,
        args.date,
        args.time,
        timezone_name=args.timezone,
        elevation_m=args.elevation_m,
        model=args.model,
        endpoint=args.endpoint,
        max_expected_height_m=args.max_expected_height_m,
        extend_above_model_top=not args.no_isa_extension,
    )

    env = result.environment
    print(f"Created RocketPy environment for {args.date} {args.time} {args.timezone}")
    print(f"Launch site: lat={env.latitude:.6f}, lon={env.longitude:.6f}, elevation={env.elevation:.1f} m")
    print(f"Profile top: {env.max_expected_height:.1f} m ASL")
    print(f"Open-Meteo model top before extension: {result.model_top_height_m:.1f} m ASL")
    print(f"Pressure points: {len(result.pressure)}")
    print(f"Temperature points: {len(result.temperature)}")
    print(f"Wind points: {len(result.wind_u)}")
    print(f"Request URL: {result.request_url}")

    if args.info:
        env.info()


if __name__ == "__main__":
    main()
