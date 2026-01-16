import simplekml
import numpy as np
from datetime import datetime, timedelta, timezone, time
import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog
import threading
import netCDF4
from timezonefinder import TimezoneFinder
import pytz
import requests
import time
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import os
import sys
import subprocess
import json
import tempfile
import shutil
from PIL import Image, ImageTk
import csv

# --- Data Fetching and Calculation Functions ---
def fetch_ground_elevation(lat, lon, log_callback):
    """Fetches ground elevation from the Open-Elevation API."""
    log_callback(f"Fetching ground elevation for lat={lat}, lon={lon}...")
    try:
        response = requests.get(f"https://api.open-elevation.com/api/v1/lookup?locations={lat},{lon}", timeout=10)
        response.raise_for_status()
        elevation = response.json()['results'][0]['elevation']
        log_callback(f"Ground elevation is {elevation:.2f} m")
        return elevation
    except Exception as e:
        log_callback(f"Could not fetch ground elevation: {e}. Defaulting to 0 m.")
        return 0

def calculate_air_density(pressure_pa, temperature_k):
    """Calculates air density using the Ideal Gas Law from forecast data."""
    R_SPECIFIC = 287.058
    if temperature_k <= 0: return 1.225
    return pressure_pa / (R_SPECIFIC * temperature_k)

def calculate_descent_rate(mass_kg, parachute_cd, parachute_area_m2, air_density):
    """Calculates the terminal velocity (descent rate) of the parachute."""
    g = 9.81
    if air_density <= 0 or parachute_cd <= 0 or parachute_area_m2 <=0: return 10
    return np.sqrt((2 * mass_kg * g) / (air_density * parachute_cd * parachute_area_m2))

def fetch_model_dataset(model_name, log_callback, max_attempts=10, base_delay=2):
    """
    Fetches the latest dataset for a given model from NOAA's server.
    This implementation is based on the robust backward-searching logic found in
    the provided fetchers.py file.
    """
    MODEL_CONFIGS = {
        "gfs": {
            # *** FIX: Pointing to the 1-hour resolution GFS dataset ***
            "url_template": "https://nomads.ncep.noaa.gov/dods/gfs_0p25_1hr/gfs{date}/gfs_0p25_1hr_{hour_str}",
            "update_interval_hours": 6,
            "hour_logic": lambda dt: f"{6 * (dt.hour // 6):02d}z"
        },
        "nam": {
            "url_template": "https://nomads.ncep.noaa.gov/dods/nam/nam{date}/nam_conusnest_{hour_str}",
            "update_interval_hours": 6,
            "hour_logic": lambda dt: f"{6 * (dt.hour // 6):02d}z"
        },
        "rap": {
            "url_template": "https://nomads.ncep.noaa.gov/dods/rap/rap{date}/rap_{hour_str}",
            "update_interval_hours": 1,
            "hour_logic": lambda dt: f"{dt.hour:02d}z"
        },
        "hiresw": {
            "url_template": "https://nomads.ncep.noaa.gov/dods/hiresw/hiresw{date}/hiresw_conusarw_{hour_str}",
            "update_interval_hours": 12,
            "hour_logic": lambda dt: "12z"
        }
    }

    if model_name not in MODEL_CONFIGS:
        raise ValueError(f"Model '{model_name}' is not supported.")

    config = MODEL_CONFIGS[model_name]
    time_attempt = datetime.now(tz=timezone.utc)
    
    for attempt_count in range(max_attempts):
        time_attempt -= timedelta(hours=config["update_interval_hours"])
        date_str = f"{time_attempt.year:04d}{time_attempt.month:02d}{time_attempt.day:02d}"
        hour_str = config["hour_logic"](time_attempt)
        file_url = config["url_template"].format(date=date_str, hour_str=hour_str)
        
        dataset = None
        try:
            log_callback(f"Attempt {attempt_count + 1}/{max_attempts}: Connecting to {model_name.upper()} at {file_url}")
            dataset = netCDF4.Dataset(file_url)
            _ = dataset.variables['lat'][:]
            log_callback(f"Successfully connected to {model_name.upper()} dataset.")
            return dataset
        except OSError as e:
            if dataset:
                dataset.close()
            log_callback(f"Failed. Trying previous forecast run. Error: {e}")
            time.sleep(base_delay ** (attempt_count + 1))
            
    raise ConnectionError(f"Unable to load latest weather data for {model_name.upper()} after {max_attempts} attempts.")

def get_all_wind_profiles_from_model(model_name, lat, lon, apogee_m, utc_dates_needed, log_callback, abort_event):
    """Fetches and processes wind profiles, checking for abort signal."""
    if abort_event.is_set(): return []
    dataset = None
    try:
        dataset = fetch_model_dataset(model_name, log_callback)
        if abort_event.is_set(): return []
        lats, lons, time_var = dataset.variables['lat'][:], dataset.variables['lon'][:], dataset.variables['time']
        forecast_datetimes = [datetime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, tzinfo=timezone.utc) for dt in netCDF4.num2date(time_var[:], units=time_var.units, calendar=time_var.calendar if 'calendar' in time_var.ncattrs() else 'standard')]
        if not any(d in {dt.date() for dt in forecast_datetimes} for d in utc_dates_needed):
            raise OSError("Date not covered by the latest forecast run.")
        input_lon = lon + 360 if lons.min() >= 0 and lon < 0 else lon
        lat_idx, lon_idx = np.abs(lats - lat).argmin(), np.abs(lons - input_lon).argmin()
        all_profiles = []
        level_var_name = 'lev' if 'lev' in dataset.variables else 'isobaric'
        pressure_levels_hpa = dataset.variables[level_var_name][:]
        for time_idx, forecast_dt in enumerate(forecast_datetimes):
            if abort_event.is_set(): return all_profiles
            if forecast_dt.date() not in utc_dates_needed: continue
            profile_data = {var: dataset.variables[f'{var}prs'][time_idx, :, lat_idx, lon_idx] for var in ['hgt', 'ugrd', 'vgrd', 'tmp']}
            profile = np.array(list(zip(profile_data['hgt'], profile_data['ugrd'], profile_data['vgrd'], profile_data['tmp'], pressure_levels_hpa)))
            if not np.all(np.isfinite(profile)):
                log_callback(f"Warning: Corrupt data found for forecast time {forecast_dt}. Skipping this time step.")
                continue
            profile = profile[profile[:, 0].argsort()]
            max_forecast_alt = profile[-1, 0]
            sim_altitudes = np.linspace(0, max_forecast_alt, num=100)
            interp_data = [np.interp(sim_altitudes, profile[:, 0], profile[:, i]) for i in range(1, 5)]
            sim_profile = np.array(list(zip(sim_altitudes, *interp_data)))
            all_profiles.append((forecast_dt, sim_profile, max_forecast_alt))
        log_callback(f"Successfully processed {len(all_profiles)} atmospheric profiles.")
        return all_profiles
    except Exception as e:
        log_callback(f"An error occurred while processing {model_name.upper()} data: {e}")
        return []
    finally:
        if dataset: dataset.close()

def calculate_landing_position(params):
    launch_lat, launch_lon, mass_kg, sim_profile = params['launch_lat'], params['launch_lon'], params['mass_kg'], params['sim_profile']
    apogee_m = params['apogee_m']
    max_forecast_alt = sim_profile[-1, 0]
    if apogee_m > max_forecast_alt:
        apogee_row = np.array([apogee_m] + list(sim_profile[-1, 1:]))
        sim_profile = np.vstack([sim_profile, apogee_row])
    altitudes = np.linspace(0, apogee_m, 100)
    final_sim_profile = np.array([altitudes] + [np.interp(altitudes, sim_profile[:, 0], sim_profile[:, i]) for i in range(1, 5)]).T
    trajectory_path = [(launch_lon, launch_lat, apogee_m)]
    current_lat, current_lon = launch_lat, launch_lon
    time_data, speed_data, total_time = [0.0], [], 0.0
    touchdown_velocity = None
    main_deployment_speed = 0
    time_to_main_deployment = 0
    time_to_touchdown = 0

    for i in range(len(altitudes) - 1, 0, -1):
        alt_top_msl, alt_bottom_msl = altitudes[i], altitudes[i-1]
        altitude_step, current_alt_msl = alt_top_msl - alt_bottom_msl, (alt_top_msl + alt_bottom_msl) / 2
        is_main_chute_active = params['use_main_chute'] and current_alt_msl < params['main_deploy_alt_msl']

        if is_main_chute_active and main_deployment_speed == 0:
            time_to_main_deployment = total_time
            drogue_params = params['drogue_chute']
            if drogue_params['fixed_rate']:
                main_deployment_speed = drogue_params['fixed_rate']
            else:
                current_temp_k = np.interp(current_alt_msl, final_sim_profile[:, 0], final_sim_profile[:, 3])
                current_pressure_hpa = np.interp(current_alt_msl, final_sim_profile[:, 0], final_sim_profile[:, 4])
                air_density = calculate_air_density(current_pressure_hpa * 100, current_temp_k)
                main_deployment_speed = calculate_descent_rate(mass_kg, drogue_params['cd'], drogue_params['area'], air_density)

        chute_params = params['main_chute'] if is_main_chute_active else params['drogue_chute']
        if chute_params['fixed_rate']:
            descent_rate = chute_params['fixed_rate']
        else:
            current_temp_k = np.interp(current_alt_msl, final_sim_profile[:, 0], final_sim_profile[:, 3])
            current_pressure_hpa = np.interp(current_alt_msl, final_sim_profile[:, 0], final_sim_profile[:, 4])
            air_density = calculate_air_density(current_pressure_hpa * 100, current_temp_k)
            descent_rate = calculate_descent_rate(mass_kg, chute_params['cd'], chute_params['area'], air_density)
        
        speed_data.append(descent_rate)
        if descent_rate <= 0: continue
        touchdown_velocity = descent_rate
        time_in_layer = altitude_step / descent_rate
        total_time += time_in_layer; time_data.append(total_time)
        avg_u, avg_v = (final_sim_profile[i, 1] + final_sim_profile[i-1, 1]) / 2, (final_sim_profile[i, 2] + final_sim_profile[i-1, 2]) / 2
        drift_x, drift_y = avg_u * time_in_layer, avg_v * time_in_layer
        current_lat += (drift_y / 111132.954)
        current_lon += (drift_x / (111320 * np.cos(np.radians(current_lat))))
        trajectory_path.append((current_lon, current_lat, alt_bottom_msl))
    
    time_to_touchdown = total_time
    speed_data.append(speed_data[-1] if speed_data else 10)
    return current_lat, current_lon, trajectory_path, time_data, speed_data, touchdown_velocity, main_deployment_speed, time_to_main_deployment, time_to_touchdown

def create_kml_file(predictions, launch_lat, launch_lon, log_callback):
    filename = f"landing_predictions_{datetime.now().strftime('%Y%m%d_%H%M%S')}.kml"
    kml = simplekml.Kml()
    kml.newpoint(name="Launch Site", coords=[(launch_lon, launch_lat)])
    num_predictions, num_styles = len(predictions), 10
    styles = []
    for i in range(num_styles):
        style = simplekml.Style()
        ratio = i / (num_styles - 1) if num_styles > 1 else 0
        red, blue = int(255 * ratio), int(255 * (1 - ratio))
        color_hex = f"ff{blue:02x}00{red:02x}"
        style.linestyle.color, style.iconstyle.color, style.linestyle.width = color_hex, color_hex, 3
        styles.append(style)
    for i, (time_str, lat, lon, trajectory, *_) in enumerate(predictions):
        style_index = int((i / (num_predictions - 1)) * (num_styles - 1)) if num_predictions > 1 else 0
        pnt = kml.newpoint(name=f"Landing @ {time_str}", coords=[(lon, lat)])
        pnt.style.iconstyle.icon.href = 'http://googleusercontent.com/maps/google.com/0'
        pnt.style = styles[style_index]
        ls = kml.newlinestring(name=f"Trajectory for {time_str}", coords=trajectory)
        ls.altitudemode, ls.extrude, ls.style = simplekml.AltitudeMode.absolute, 0, styles[style_index]
    kml.save(filename)
    log_callback(f"KML file '{filename}' created successfully.")
    return filename

def create_descent_plot(predictions, log_callback):
    if not predictions: return
    # Unpack carefully to get the correct data
    time_str, _, _, _, time_data, speed_data, *_ = predictions[0]
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, speed_data)
    plt.xlabel("Time Since Apogee (s)"), plt.ylabel("Descent Speed (m/s)"), plt.title(f"Descent Profile (Forecast: {time_str})"), plt.grid(True)
    filename = f"descent_profile_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    plt.savefig(filename), plt.close(), log_callback(f"Descent profile plot saved as '{filename}'.")

def create_descent_csv(predictions, log_callback):
    """Saves the descent profile data (time vs. speed) for all forecasts to a CSV file."""
    if not predictions: return
    filename = f"descent_profiles_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    try:
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['forecast_time_local', 'time_since_apogee_s', 'descent_speed_ms'])
            
            for prediction in predictions:
                # Unpack carefully to get the correct data
                time_str, _, _, _, time_data, speed_data, *_ = prediction
                min_len = min(len(time_data), len(speed_data))
                for i in range(min_len):
                    writer.writerow([time_str, f"{time_data[i]:.2f}", f"{speed_data[i]:.2f}"])
        log_callback(f"Descent data CSV file '{filename}' created successfully.")
    except Exception as e:
        log_callback(f"Error creating CSV file: {e}")

class LandingPredictorApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Rocket Landing Predictor")
        try: self.state('zoomed')
        except tk.TclError: self.attributes('-fullscreen', True)
        
        self.style = ttk.Style(self)
        self.style.theme_use('clam')
        self.style.configure("TLabel", padding=5)
        self.style.configure("TEntry", padding=5)
        self.style.configure("TButton", padding=5)
        self.style.configure("TCheckbutton", padding=5)
        self.style.configure("TLabelframe.Label", padding=5, font=("TkDefaultFont", 10, "bold"))
        self.style.configure("Green.Horizontal.TProgressbar", background='green')

        self.last_kml_file, self.current_units, self.all_simulation_profiles = None, "Metric", []
        self.wind_plot_image_paths = []
        self.temp_dir = tempfile.mkdtemp()
        
        self.prediction_thread = None
        self.abort_event = threading.Event()
        self.is_closing = False
        
        # --- NEW: Control variable for log visibility ---
        self.log_visible_var = tk.BooleanVar(value=True)
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.setup_menu()
        self.setup_ui()
        self.setup_plots()

    def on_closing(self):
        if self.prediction_thread and self.prediction_thread.is_alive():
            self.abort_prediction()
            self.is_closing = True
            self.log("Shutdown requested. Waiting for current process to complete...")
            self.after(200, self.on_closing)
            return
        try:
            if os.path.exists(self.temp_dir):
                shutil.rmtree(self.temp_dir)
        except Exception as e:
            print(f"Error cleaning up temp directory: {e}")
        self.destroy()

    def setup_menu(self):
        menubar = tk.Menu(self)
        self.config(menu=menubar)
        
        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="File", menu=file_menu)
        file_menu.add_command(label="Save Configuration", command=self.save_config)
        file_menu.add_command(label="Load Configuration", command=self.load_config)
        file_menu.add_separator(), file_menu.add_command(label="Exit", command=self.on_closing)
        
        settings_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Settings", menu=settings_menu)
        self.unit_var = tk.StringVar(value="Metric")
        unit_menu = tk.Menu(settings_menu, tearoff=0)
        settings_menu.add_cascade(label="Units", menu=unit_menu)
        unit_menu.add_radiobutton(label="Metric", variable=self.unit_var, value="Metric", command=self.on_unit_change)
        unit_menu.add_radiobutton(label="Imperial", variable=self.unit_var, value="Imperial", command=self.on_unit_change)
        
        # --- NEW: View Menu for toggling the log ---
        view_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="View", menu=view_menu)
        view_menu.add_checkbutton(label="Show Log Panel", variable=self.log_visible_var, command=self._toggle_log_visibility)
        
    def setup_ui(self):
        main_frame = ttk.Frame(self)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        left_frame = ttk.Frame(main_frame, width=450)
        left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left_frame.pack_propagate(False)

        top_left_frame = ttk.Frame(left_frame)
        top_left_frame.pack(fill=tk.BOTH, expand=True)
        self.input_canvas = tk.Canvas(top_left_frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(top_left_frame, orient="vertical", command=self.input_canvas.yview)
        self.scrollable_frame = ttk.Frame(self.input_canvas, padding=(10,5))
        self.scrollable_frame_window = self.input_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.scrollable_frame.bind("<Configure>", lambda e: self.input_canvas.configure(scrollregion=self.input_canvas.bbox("all")))
        self.input_canvas.bind("<Configure>", lambda e: self.input_canvas.itemconfig(self.scrollable_frame_window, width=e.width))
        self.input_canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self._bind_scroll_to_widgets(self.scrollable_frame, self.input_canvas)

        bottom_left_frame = ttk.Frame(left_frame, padding=5)
        bottom_left_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        button_frame = ttk.Frame(bottom_left_frame)
        button_frame.pack(fill='x', expand=True)
        self.run_button = ttk.Button(button_frame, text="Run Forecasts", command=self.start_prediction_thread)
        self.run_button.pack(side=tk.LEFT, pady=5, padx=5, fill='x', expand=True)
        
        self.abort_button = ttk.Button(button_frame, text="Abort", command=self.abort_prediction, state="disabled")
        self.abort_button.pack(side=tk.LEFT, pady=5, padx=5, fill='x', expand=True)

        self.open_kml_button = ttk.Button(bottom_left_frame, text="Open Latest KML File", command=self.open_kml_file, state="disabled")
        self.open_kml_button.pack(pady=5, padx=5, fill='x')

        self.progress_label = ttk.Label(bottom_left_frame, text="", anchor="center")
        self.progress_label.pack(pady=(5,0), padx=5, fill='x', expand=True)

        self.progressbar = ttk.Progressbar(bottom_left_frame, orient="horizontal", mode="determinate")
        self.progressbar.pack(pady=(0,5), padx=5, fill='x', expand=True)
        
        # --- MODIFIED: Paned window and log frame are now instance variables ---
        self.right_paned_window = ttk.PanedWindow(main_frame, orient=tk.VERTICAL)
        self.right_paned_window.pack(fill=tk.BOTH, expand=True)
        plots_pane = ttk.PanedWindow(self.right_paned_window, orient=tk.HORIZONTAL)
        self.right_paned_window.add(plots_pane, weight=2)
        
        dispersion_plot_frame = ttk.LabelFrame(plots_pane, text="Landing Dispersion", padding=(10, 5))
        self.fig_disp, self.ax_disp = plt.subplots(figsize=(5, 5), dpi=100)
        self.canvas_disp = FigureCanvasTkAgg(self.fig_disp, master=dispersion_plot_frame)
        self.canvas_disp.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        plots_pane.add(dispersion_plot_frame, weight=1)
        wind_plot_frame_container = ttk.Frame(plots_pane)
        self.wind_profile_slider = ttk.Scale(wind_plot_frame_container, from_=0, to=0, orient=tk.HORIZONTAL, command=self.on_wind_profile_slider_change, state="disabled")
        self.wind_profile_slider.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(0, 5))
        wind_plot_frame = ttk.LabelFrame(wind_plot_frame_container, text="Wind Profile", padding=(10, 5))
        wind_plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.wind_plot_label = ttk.Label(wind_plot_frame)
        self.wind_plot_label.pack(fill=tk.BOTH, expand=True)
        self.wind_plot_label.bind('<Configure>', self._update_wind_plot_image)
        plots_pane.add(wind_plot_frame_container, weight=1)
        
        self.log_frame = ttk.LabelFrame(self.right_paned_window, text="Log", padding=(10, 5))
        self.log_text = scrolledtext.ScrolledText(self.log_frame, wrap=tk.WORD, height=10)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        self.right_paned_window.add(self.log_frame, weight=1)
        
        self.create_input_fields()
    
    # --- NEW: Method to toggle log visibility ---
    def _toggle_log_visibility(self):
        if self.log_visible_var.get():
            try:
                self.right_paned_window.add(self.log_frame, weight=1)
            except tk.TclError: # Already exists
                pass
        else:
            try:
                self.right_paned_window.forget(self.log_frame)
            except tk.TclError: # Already forgotten
                pass
    
    # ... (the rest of the methods follow)
    def _bind_scroll_to_widgets(self, widget, canvas):
        widget.bind('<Enter>', lambda e: canvas.bind_all("<MouseWheel>", lambda event: self._on_mousewheel(event, canvas)))
        widget.bind('<Leave>', lambda e: self.unbind_all("<MouseWheel>"))
    def _on_mousewheel(self, event, canvas):
        widget_under_mouse = self.winfo_containing(event.x_root, event.y_root)
        if widget_under_mouse is None: return
        parent = widget_under_mouse
        while parent is not None:
            if parent == canvas:
                canvas.yview_scroll(int(-1*(event.delta/120)), "units")
                return
            parent = parent.master
    def create_input_fields(self):
        self.inputs = {}
        general_frame = ttk.LabelFrame(self.scrollable_frame, text="General Parameters", padding=(10, 5))
        general_frame.pack(padx=10, pady=10, fill="x")
        gen_params = {"Launch Latitude": ("Launch Latitude", "39.5296"), "Launch Longitude": ("Launch Longitude", "-119.8138"), 
                      "Launch Date": ("Launch Date (YYYY-MM-DD)", datetime.now().strftime('%Y-%m-%d')),
                      "Apogee": ("Apogee (m AGL)", "3000"), "Rocket Mass": ("Rocket Mass (kg)", "5.0")}
        for i, (key, (text, val)) in enumerate(gen_params.items()): self.create_entry(general_frame, key, text, val, i)
        time_range_frame = ttk.LabelFrame(general_frame, text="Time Range (Local)", padding=(10,5))
        time_range_frame.grid(row=len(gen_params), column=0, columnspan=2, sticky='ew', pady=5, padx=5)
        self.create_entry(time_range_frame, "Start Time", "Start Time (Local, HH:MM)", "00:00", 0)
        self.create_entry(time_range_frame, "End Time", "End Time (Local, HH:MM)", "23:59", 1)
        model_label = ttk.Label(general_frame, text="Weather Model")
        model_label.grid(row=len(gen_params)+1, column=0, sticky="w", padx=5, pady=2)
        self.model_selection = ttk.Combobox(general_frame, values=["GFS", "NAM", "RAP", "HIRESW"], state="readonly")
        self.model_selection.grid(row=len(gen_params)+1, column=1, sticky="ew", padx=5, pady=2)
        self.model_selection.set("GFS")
        ttk.Label(general_frame, text="Note: NAM, RAP, and HIRESW models only cover North America.", font=('TkDefaultFont', 8, 'italic')).grid(row=len(gen_params) + 2, column=0, columnspan=2, sticky="w", padx=5)
        drogue_frame = ttk.LabelFrame(self.scrollable_frame, text="Drogue Parachute", padding=(10, 5))
        drogue_frame.pack(padx=10, pady=10, fill="x")
        drogue_params = {"Drogue Descent Rate": ("Descent Rate (m/s)", ""), "Drogue Cd": ("Parachute Cd", "1.5"), "Drogue Area": ("Parachute Area (m²)", "0.5")}
        for i, (key, (text, val)) in enumerate(drogue_params.items()): self.create_entry(drogue_frame, key, text, val, i)
        main_frame = ttk.LabelFrame(self.scrollable_frame, text="Main Parachute", padding=(10, 5))
        main_frame.pack(padx=10, pady=10, fill="x")
        self.main_chute_enabled = tk.BooleanVar()
        self.main_chute_check = ttk.Checkbutton(main_frame, text="Enable Main Parachute", variable=self.main_chute_enabled, command=self.toggle_main_chute_fields)
        self.main_chute_check.grid(row=0, column=0, columnspan=2, sticky='w', pady=5)
        self.main_chute_fields = []
        main_params = {"Main Deployment Altitude": ("Deployment Altitude (m AGL)", "700"), "Main Descent Rate": ("Descent Rate (m/s)", ""), 
                       "Main Cd": ("Parachute Cd", "2.2"), "Main Area": ("Parachute Area (m²)", "1.5")}
        for i, (key, (text, val)) in enumerate(main_params.items(), 1): self.create_entry(main_frame, key, text, val, i, fields_list=self.main_chute_fields)
        self.toggle_main_chute_fields()
    def setup_plots(self):
        self.ax_disp.clear()
        self.update_dispersion_plot([], 0)
        self.wind_plot_label.config(image='')
        self.wind_plot_label.image = None
        
    def update_dispersion_plot(self, predictions, launch_lat):
        self.ax_disp.clear()
        self.ax_disp.set_title("Predicted Landing Dispersion")

        units = self.unit_var.get()
        dist_unit_str = "(ft)" if units == "Imperial" else "(m)"
        dist_factor = 3.28084 if units == "Imperial" else 1.0

        self.ax_disp.set_xlabel(f"East of Launch Site {dist_unit_str}")
        self.ax_disp.set_ylabel(f"North of Launch Site {dist_unit_str}")
        self.ax_disp.grid(True)

        # Plot launch site first
        self.ax_disp.plot(0, 0, 'k+', markersize=10, label='Launch Site')

        # Store all points (including launch site) in METERS to find the bounding box
        east_points_m = [0]
        north_points_m = [0]
        
        if predictions:
            for _, lat, lon, *_ in predictions:
                delta_lat = lat - launch_lat
                try:
                    launch_lon = float(self.inputs["Launch Longitude"][1].get())
                    delta_lon = lon - launch_lon
                except (ValueError, KeyError):
                    delta_lon = 0
                
                north_points_m.append(delta_lat * 111132.954)
                east_points_m.append(delta_lon * 111320 * np.cos(np.radians(launch_lat)))

        # Plot the landing points
        # We plot using the stored meter values converted to display units
        for i in range(len(predictions)):
            ratio = i / (len(predictions) - 1) if len(predictions) > 1 else 0
            time_str = predictions[i][0]
            
            dist_east_display = east_points_m[i+1] * dist_factor # (index 0 is the launch site)
            dist_north_display = north_points_m[i+1] * dist_factor
            
            self.ax_disp.scatter(dist_east_display, dist_north_display, color=plt.cm.jet(ratio))
            self.ax_disp.annotate(time_str.split(' ')[0], (dist_east_display, dist_north_display), textcoords="offset points", xytext=(5,-5), ha='left', fontsize=8)

        # Determine the bounding box of all points in meters
        min_east_m, max_east_m = min(east_points_m), max(east_points_m)
        min_north_m, max_north_m = min(north_points_m), max(north_points_m)

        # Calculate the range in each direction
        range_east_m = max_east_m - min_east_m
        range_north_m = max_north_m - min_north_m

        # Find the largest range and use it to make the plot square
        max_range_m = max(range_east_m, range_north_m)
        if max_range_m == 0: max_range_m = 200 # Default size for a plot with one point

        # Find the center of the bounding box
        center_east_m = (max_east_m + min_east_m) / 2
        center_north_m = (max_north_m + min_north_m) / 2

        # Set the limits, converting from meters to the display unit with 10% padding
        half_range_display = (max_range_m * 1.1 / 2) * dist_factor
        center_east_display = center_east_m * dist_factor
        center_north_display = center_north_m * dist_factor
        
        self.ax_disp.set_xlim(center_east_display - half_range_display, center_east_display + half_range_display)
        self.ax_disp.set_ylim(center_north_display - half_range_display, center_north_display + half_range_display)
        
        # This ensures the scale is 1:1, making the square limits appear correctly
        self.ax_disp.set_aspect('equal', adjustable='box')
        
        self.ax_disp.legend()
        self.canvas_disp.draw()        
    def _update_wind_plot_image(self, event=None):
        if not self.wind_plot_image_paths: return
        idx = int(round(float(self.wind_profile_slider.get())))
        if 0 <= idx < len(self.wind_plot_image_paths):
            filepath = self.wind_plot_image_paths[idx]
            width, height = self.wind_plot_label.winfo_width(), self.wind_plot_label.winfo_height()
            if width > 1 and height > 1:
                try:
                    img = Image.open(filepath)
                    img_resized = img.resize((width, height), Image.Resampling.LANCZOS)
                    photo = ImageTk.PhotoImage(img_resized)
                    self.wind_plot_label.config(image=photo)
                    self.wind_plot_label.image = photo
                except Exception as e:
                    self.log(f"Error loading plot image: {e}")
    def on_wind_profile_slider_change(self, event=None):
        self._update_wind_plot_image()
    def generate_wind_plot_image(self, index, sim_profile, local_time, max_forecast_alt):
        fig, ax = plt.subplots(figsize=(5, 5), dpi=100)
        altitudes, u_winds, v_winds = sim_profile[:, 0], sim_profile[:, 1], sim_profile[:, 2]
        wind_speed = np.sqrt(u_winds**2 + v_winds**2)
        wind_dir = (270 - np.rad2deg(np.arctan2(v_winds, u_winds))) % 360
        speed_unit_str, alt_unit_str, factor = ("(m/s)", "(m MSL)", 1.0)
        if self.unit_var.get() == "Imperial":
            speed_unit_str, alt_unit_str, factor = ("(ft/s)", "(ft MSL)", 3.28084)
        ax.set_ylabel(f"Altitude {alt_unit_str}")
        ax.set_xlabel(f"Speed {speed_unit_str}")
        ax.plot(wind_speed * factor, altitudes * factor, color='black', linewidth=2)
        ax.set_ylim(0, max_forecast_alt * factor)
        if np.any(wind_speed):
            ax.set_xlim(left=0, right=np.max(wind_speed) * factor * 1.1)
        apogee_val_str = self.inputs["Apogee"][1].get()
        if apogee_val_str:
            apogee_val_m = float(apogee_val_str) * (1/3.28084 if self.current_units == "Imperial" else 1.0)
            apogee_display = apogee_val_m * factor
            ax.axhline(y=apogee_display, color='purple', linestyle=':', label=f'Apogee')
        ax.set_title(f"Wind Profile ({local_time.strftime('%H:%M Local')})")
        ax.grid(True)
        ax2 = ax.twiny()
        mask = wind_speed > 1
        ax2.plot(wind_dir[mask], altitudes[mask] * factor, color='red', linestyle=':', linewidth=2, marker='.', markersize=2)
        ax2.set_xlabel("Direction (deg from N)", color='red')
        ax2.tick_params(axis='x', labelcolor='red')
        ax2.set_xlim(0, 360)
        fig.tight_layout()
        filepath = os.path.join(self.temp_dir, f"wind_profile_{index}.png")
        fig.savefig(filepath)
        plt.close(fig)
        return filepath
    def create_entry(self, parent, key, text, default_val, row, column_offset=0, fields_list=None):
        label = ttk.Label(parent, text=text)
        label.grid(row=row, column=column_offset, sticky="w", padx=5, pady=2)
        entry = ttk.Entry(parent, width=20)
        entry.grid(row=row, column=column_offset + 1, sticky="ew", padx=5, pady=2)
        parent.columnconfigure(column_offset + 1, weight=1)
        entry.insert(0, default_val)
        self.inputs[key] = (label, entry)
        if fields_list is not None: fields_list.extend([label, entry])
    def toggle_main_chute_fields(self):
        state = "normal" if self.main_chute_enabled.get() else "disabled"
        for widget in self.main_chute_fields: widget.config(state=state)
    def log(self, message):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END), self.update_idletasks()
    def _update_progress(self, value, is_complete=False):
        self.progressbar['value'] = value
        if is_complete:
            self.progressbar['style'] = "Green.Horizontal.TProgressbar"
        else:
            self.progressbar['style'] = "Horizontal.TProgressbar"
    def _update_status(self, text):
        self.progress_label.config(text=text)
    def start_prediction_thread(self):
        self.abort_event.clear()
        self.run_button.config(state="disabled")
        self.abort_button.config(state="normal")
        self.open_kml_button.config(state="disabled")
        self.log("Starting prediction..."), self.setup_plots()
        self._update_progress(0)
        self._update_status("Initializing...")
        self.prediction_thread = threading.Thread(target=self.run_prediction, daemon=True)
        self.prediction_thread.start()
    def abort_prediction(self):
        if self.prediction_thread and self.prediction_thread.is_alive():
            self.log("Abort signal sent...")
            self._update_status("Aborting...")
            self.abort_event.set()
            self.abort_button.config(state="disabled")
    def open_kml_file(self):
        if self.last_kml_file and os.path.exists(self.last_kml_file):
            try:
                if sys.platform == "win32":
                    os.startfile(self.last_kml_file)
                elif sys.platform == "darwin":
                    subprocess.call(["open", self.last_kml_file])
                else: # linux
                    subprocess.call(["xdg-open", self.last_kml_file])
            except Exception as e:
                self.log(f"Error opening KML file: {e}")
        else:
            self.log("No KML file generated yet, or the last file was moved/deleted.")
    def get_params_in_metric(self):
        params, units = {}, self.unit_var.get()
        ft_to_m, lbs_to_kg, sqft_to_sqm = 0.3048, 0.453592, 0.092903
        def get(key, factor=1.0):
            val_str = self.inputs[key][1].get(); return float(val_str) * factor if val_str else None
        params['launch_lat'], params['launch_lon'] = get("Launch Latitude"), get("Launch Longitude")
        factor = ft_to_m if units == "Imperial" else 1.0
        params.update({
            'apogee_m_agl': get("Apogee", factor), 'mass_kg': get("Rocket Mass", lbs_to_kg if units == "Imperial" else 1.0),
            'drogue_chute': {'fixed_rate': get("Drogue Descent Rate", factor), 'cd': get("Drogue Cd"), 'area': get("Drogue Area", sqft_to_sqm if units == "Imperial" else 1.0)},
            'main_deploy_alt_agl': get("Main Deployment Altitude", factor) if self.main_chute_enabled.get() else 0,
            'main_chute': {'fixed_rate': get("Main Descent Rate", factor), 'cd': get("Main Cd"), 'area': get("Main Area", sqft_to_sqm if units == "Imperial" else 1.0)}
        })
        params['use_main_chute'] = self.main_chute_enabled.get()
        return params

    def run_prediction(self):
        try:
            self.after(0, self._update_progress, 0)
            self.after(0, self._update_status, "Validating inputs...")
            params = self.get_params_in_metric()
            lat, lon = params['launch_lat'], params['launch_lon']
            if not -90 <= lat <= 90: raise ValueError(f"Invalid Latitude: {lat}. Must be between -90 and 90.")
            if not -180 <= lon <= 180: raise ValueError(f"Invalid Longitude: {lon}. Must be between -180 and 180.")
            if self.abort_event.is_set(): return

            self.after(0, self._update_status, "Fetching ground elevation...")
            ground_elevation_m = fetch_ground_elevation(lat, lon, self.log)
            self.after(0, self._update_progress, 5)
            params['apogee_m'] = params['apogee_m_agl'] + ground_elevation_m
            if params['use_main_chute']:
                params['main_deploy_alt_msl'] = params['main_deploy_alt_agl'] + ground_elevation_m
                self.log(f"Main chute deployment altitude set to {params['main_deploy_alt_msl']:.2f} m MSL.")
            if self.abort_event.is_set(): return

            self.after(0, self._update_progress, 10)
            self.after(0, self._update_status, "Fetching weather model data...")
            try:
                local_tz_name = TimezoneFinder().timezone_at(lng=params['launch_lon'], lat=params['launch_lat'])
                local_tz = pytz.timezone(local_tz_name or 'UTC')
                self.log(f"Determined local timezone: {local_tz.zone}")
            except ValueError:
                self.log("Warning: Could not determine timezone. Using UTC for display.")
                local_tz = pytz.utc
            launch_date = datetime.strptime(self.inputs["Launch Date"][1].get(), '%Y-%m-%d').date()
            start_time_str, end_time_str = self.inputs["Start Time"][1].get(), self.inputs["End Time"][1].get()
            start_dt_local_naive = datetime.combine(launch_date, datetime.strptime(start_time_str, "%H:%M").time())
            end_dt_local_naive = datetime.combine(launch_date, datetime.strptime(end_time_str, "%H:%M").time())
            if end_dt_local_naive < start_dt_local_naive:
                end_dt_local_naive += timedelta(days=1)
                self.log("Overnight time window detected. End time adjusted to the next day.")
            start_dt_utc, end_dt_utc = start_dt_local_naive.astimezone(pytz.utc), end_dt_local_naive.astimezone(pytz.utc)
            utc_dates_needed = {d.date() for d in [start_dt_utc, end_dt_utc]}
            self.log(f"Filtering forecasts between {start_time_str} and {end_time_str} Local Time.")
            model_name = self.model_selection.get().lower()
            all_profiles_raw = get_all_wind_profiles_from_model(model_name, lat, lon, params['apogee_m'], utc_dates_needed, self.log, self.abort_event)
            self.after(0, self._update_progress, 50)
            if self.abort_event.is_set() or not all_profiles_raw: return

            predictions, self.all_simulation_profiles, self.wind_plot_image_paths = [], [], []
            units = self.unit_var.get()
            speed_unit_str, speed_factor = ("ft/s", 3.28084) if units == "Imperial" else ("m/s", 1.0)
            for utc_time, sim_profile, max_alt in all_profiles_raw:
                if not (start_dt_utc <= utc_time <= end_dt_utc): continue
                self.all_simulation_profiles.append((utc_time, sim_profile, max_alt))
            if not self.all_simulation_profiles:
                self.log(f"\nNo forecast data available in the specified time range {start_time_str} - {end_time_str}.")
                return
            num_sims = len(self.all_simulation_profiles)
            for i, (utc_time, sim_profile, max_alt) in enumerate(self.all_simulation_profiles):
                if self.abort_event.is_set(): return
                local_time = utc_time.astimezone(local_tz)
                self.after(0, self._update_status, f"Simulating for {local_time.strftime('%H:%M')} Local ({i+1}/{num_sims})")
                
                filepath = self.generate_wind_plot_image(i, sim_profile, local_time, max_alt)
                self.wind_plot_image_paths.append(filepath)
                self.log(f"\n--- Simulating for forecast time: {local_time.strftime('%Y-%m-%d %H:%M')} Local ---")
                params['sim_profile'] = sim_profile
                land_lat, land_lon, trajectory, time_data, speed_data, touchdown_vel, main_deploy_vel, time_to_main, time_to_ground = calculate_landing_position(params)
                time_str = local_time.strftime('%H:%M Local')
                predictions.append((time_str, land_lat, land_lon, trajectory, time_data, speed_data, touchdown_vel, main_deploy_vel, time_to_main, time_to_ground))
                self.log(f"Predicted Landing: Lat: {land_lat:.6f}, Lon: {land_lon:.6f}")
                if params['use_main_chute'] and main_deploy_vel > 0:
                    self.log(f"  └─ Main Chute Deployment Speed: {main_deploy_vel * speed_factor:.2f} {speed_unit_str}")
                if touchdown_vel is not None:
                    self.log(f"  └─ Touchdown Velocity: {touchdown_vel * speed_factor:.2f} {speed_unit_str}")
                if params['use_main_chute'] and time_to_main > 0:
                    self.log(f"  └─ Time to Main Deployment: {time_to_main:.1f} s")
                if time_to_ground is not None:
                    self.log(f"  └─ Time to Touchdown: {time_to_ground:.1f} s")

                self.after(0, self.update_dispersion_plot, predictions, params['launch_lat'])
                loop_progress = ((i + 1) / num_sims) * 50
                self.after(0, self._update_progress, 50 + loop_progress)
            
            self.after(0, self.wind_profile_slider.config, {"to": len(self.all_simulation_profiles) - 1, "state": "normal"})
            self.after(100, self._update_wind_plot_image)
            if predictions:
                self.last_kml_file = create_kml_file(predictions, params['launch_lat'], params['launch_lon'], self.log)
                create_descent_plot(predictions, self.log)
                create_descent_csv(predictions, self.log)
                self.open_kml_button.config(state="normal")
            
            self.after(0, lambda: self._update_progress(100, is_complete=True))
            self.after(0, self._update_status, "Complete!")
        except Exception as e:
            self.log(f"An error occurred: {e}")
            self.after(0, self._update_status, "Error!")
            self.after(0, self._update_progress, 0)
        finally:
            if self.abort_event.is_set():
                self.log("\nPrediction aborted by user.")
                self.after(0, self._update_status, "Aborted")
                self.after(0, self._update_progress, 0)
            else:
                self.log("\nPrediction run finished.")
            self.after(0, self.run_button.config, {"state": "normal"})
            self.after(0, self.abort_button.config, {"state": "disabled"})
            self.after(2000, self._update_status, "")

    def on_unit_change(self, event=None):
        new_units = self.unit_var.get()
        if new_units == self.current_units: return
        conversions = { "Apogee": (3.28084, " (ft AGL)", "(m AGL)"), "Rocket Mass": (2.20462, " (lbs)", "(kg)"),
            "Drogue Descent Rate": (3.28084, " (ft/s)", "(m/s)"), "Drogue Area": (10.7639, " (ft²)", "(m²)"),
            "Main Deployment Altitude": (3.28084, " (ft AGL)", "(m AGL)"), "Main Descent Rate": (3.28084, " (ft/s)", "(m/s)"),
            "Main Area": (10.7639, " (ft²)", "(m²)")}
        base_labels = {key: self.inputs[key][0].cget("text").split(" (")[0] for key in conversions}
        for key, (factor, imp_unit, met_unit) in conversions.items():
            label, entry = self.inputs[key]
            val_str = entry.get()
            if val_str:
                try:
                    val = float(val_str)
                    new_val = val * factor if new_units == "Imperial" else val / factor
                    entry.delete(0, tk.END)
                    entry.insert(0, f"{new_val:.2f}")
                except ValueError: pass
            label.config(text=base_labels[key] + (imp_unit if new_units == "Imperial" else met_unit))
        self.current_units = new_units
        try:
            lat = float(self.inputs["Launch Latitude"][1].get())
        except (ValueError, KeyError):
            lat = 0
        self.update_dispersion_plot([], lat)

    def load_config(self):
        filepath = filedialog.askopenfilename(filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if not filepath: return
        try:
            with open(filepath, 'r') as f: config = json.load(f)
            self.main_chute_enabled.set(config.get("main_chute_enabled", False))
            self.toggle_main_chute_fields()
            loaded_units = config.get("units", "Metric")
            if self.unit_var.get() != loaded_units:
                self.unit_var.set(loaded_units)
                self.on_unit_change()
                self.current_units = "Metric" 
            for key, (label, entry) in self.inputs.items():
                if key in config: 
                    entry.delete(0, tk.END)
                    entry.insert(0, config[key])
            self.on_unit_change()
            self.model_selection.set(config.get("model_selection", "gfs"))
            self.log(f"Configuration loaded from {filepath}")
        except Exception as e: self.log(f"Error loading config: {e}")

    def save_config(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if not filepath: return
        config = {"units": self.unit_var.get(), "main_chute_enabled": self.main_chute_enabled.get()}
        config["start_time"], config["end_time"] = self.inputs["Start Time"][1].get(), self.inputs["End Time"][1].get()
        config['model_selection'] = self.model_selection.get()
        for key, (_, entry) in self.inputs.items(): config[key] = entry.get()
        try:
            with open(filepath, 'w') as f: json.dump(config, f, indent=4)
            self.log(f"Configuration saved to {filepath}")
        except Exception as e: self.log(f"Error saving config: {e}")

if __name__ == '__main__':
    app = LandingPredictorApp()
    app.mainloop()