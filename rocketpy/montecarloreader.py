import numpy as np
import matplotlib.pyplot as plt

from matplotlib.patches import Ellipse
import csv

import pickle

with open('flight_data.pkl', 'rb') as f:
    results = pickle.load(f)
    
#print(results)

x_data = []
y_data = []
x_apogee = []
y_apogee = []
z_apogee = []

ignitionangles = []
ignitiontimings = []
ignitionvelocites = []
maxvelocity = []


def quaternion_to_angle_from_vertical(e0, e1, e2, e3):
    """
    Convert quaternion to angle from vertical (zenith angle)
    
    Parameters:
    e0, e1, e2, e3: Quaternion components
    
    Returns:
    angle_degrees: Angle from vertical in degrees
    """
    # Normalize quaternion (just in case)
    norm = np.sqrt(e0**2 + e1**2 + e2**2 + e3**2)
    e0, e1, e2, e3 = e0/norm, e1/norm, e2/norm, e3/norm
    
    # The vertical direction in body frame after rotation
    # Original vertical is [0, 0, 1] (pointing up)
    # Apply quaternion rotation to get current vertical direction
    
    # Quaternion rotation formula for vector [0, 0, 1]
    vx = 2 * (e0*e2 + e1*e3)
    vy = 2 * (e1*e2 - e0*e3)
    vz = e0**2 - e1**2 - e2**2 + e3**2
    
    # Angle from vertical is the angle between [0, 0, 1] and [vx, vy, vz]
    # cos(angle) = dot product with [0, 0, 1] = vz
    angle_rad = np.arccos(np.clip(vz, -1, 1))
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg


#print(results)
for x in results:
    if x[-1][0] > 60:
        x_data.append(x[-1][1])

        y_data.append(x[-1][2])
        ignitiontimings.append(x[0][0])
        ignitionvelocites.append(x[0][6])
        thisflightignitionangle = quaternion_to_angle_from_vertical(x[0][7],x[0][8],x[0][9],x[0][10])
        ignitionangles.append(thisflightignitionangle)
        print(thisflightignitionangle)

        apogee = 0
        apogee_x = 0
        apogee_y = 0
        apogee_index = 0
        for y in x:
            if y[3] > apogee:
                apogee = y[3]
                apogee_x = y[1]
                apogee_y = y[2]
                apogee_index = apogee_index + 1
            else:
                break
        x_apogee.append(x[apogee_index][1])
        y_apogee.append(x[apogee_index][2])
        z_apogee.append(apogee)
        #print(str(apogee) + " at " + str(apogee_index))


#MATPLOTLIB PLOTS------------------------------------------------------------------

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(10, 6))

ax1.hist(z_apogee)
ax1.set_xlabel('Apogee(m)', fontsize=12)
ax1.set_ylabel('Frequency', fontsize=12)
ax1.set_title('Apogees', fontsize=14)


ax2.hist(ignitionangles)
ax2.set_xlabel('Ignition angles (deg)', fontsize=12)
ax2.set_ylabel('Frequency', fontsize=12)
ax2.set_title('Ignition angles', fontsize=14)

ax3.hist(ignitiontimings)
ax3.set_xlabel('Ignition Timings (s)', fontsize=12)
ax3.set_ylabel('Frequency', fontsize=12)
ax3.set_title('Ignition Timings', fontsize=14)

ax4.hist(ignitionvelocites)
ax4.set_xlabel('Ignition Velocity (m/s)', fontsize=12)
ax4.set_ylabel('Frequency', fontsize=12)
ax4.set_title('Ignition Velcoties', fontsize=14)

plt.tight_layout()

plt.show()

origin_lat = 35.3466  # Replace with your launch site
origin_lon = -117.809

# Convert meters to GPS coordinates
meters_per_degree_lat = 111320
meters_per_degree_lon = 111320 * np.cos(np.radians(origin_lat))

# Landing coordinates
lat_landing = [origin_lat + (y / meters_per_degree_lat) for y in y_data]
lon_landing = [origin_lon + (x / meters_per_degree_lon) for x in x_data]

# Apogee coordinates
lat_apogee = [origin_lat + (y / meters_per_degree_lat) for y in y_apogee]
lon_apogee = [origin_lon + (x / meters_per_degree_lon) for x in x_apogee]

# Calculate statistics for landing
mean_lat_landing = np.mean(lat_landing)
mean_lon_landing = np.mean(lon_landing)
mean_x_landing = np.mean(x_data)
mean_y_landing = np.mean(y_data)
std_x_landing = np.std(x_data)
std_y_landing = np.std(y_data)

# Calculate statistics for apogee
mean_lat_apogee = np.mean(lat_apogee)
mean_lon_apogee = np.mean(lon_apogee)
mean_x_apogee = np.mean(x_apogee)
mean_y_apogee = np.mean(y_apogee)
mean_z_apogee = np.mean(z_apogee)
std_x_apogee = np.std(x_apogee)
std_y_apogee = np.std(y_apogee)
std_z_apogee = np.std(z_apogee)

# Function to create circle coordinates
def create_circle(center_lon, center_lat, radius_x, radius_y, num_points=100):
    """Create circle coordinates in lon/lat"""
    angles = np.linspace(0, 2*np.pi, num_points)
    lons = center_lon + (radius_x / meters_per_degree_lon) * np.cos(angles)
    lats = center_lat + (radius_y / meters_per_degree_lat) * np.sin(angles)
    return lons, lats



# Create KML file
kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
    <name>Rocket Flight Data</name>
    <description>Landing points and apogee locations with standard deviation ellipses</description>
    
    <!-- Style for launch site -->
    <Style id="launchStyle">
    <IconStyle>
        <color>ff00ff00</color>
        <scale>1.5</scale>
        <Icon>
        <href>http://maps.google.com/mapfiles/kml/shapes/star.png</href>
        </Icon>
    </IconStyle>
    </Style>
    
    <!-- Style for landing points -->
    <Style id="landingStyle">
      <IconStyle>
        <color>ff0000ff</color>
        <scale>1.0</scale>
        <Icon>
          <href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>
        </Icon>
      </IconStyle>
      <LabelStyle>
        <scale>0.3</scale>
        <color>ffffffff</color>
      </LabelStyle>
    </Style>
    
    <!-- Style for apogee points -->
    <Style id="apogeeStyle">
    <IconStyle>
        <color>ffff00ff</color>
        <scale>1.0</scale>
        <Icon>
        <href>http://maps.google.com/mapfiles/kml/shapes/triangle.png</href>
        </Icon>
    </IconStyle>
    <LabelStyle>
        <scale>0.3</scale>
        <color>ffffffff</color>
      </LabelStyle>
    </Style>
    
    <!-- Style for mean landing -->
    <Style id="meanLandingStyle">
    <IconStyle>
        <color>ff00ffff</color>
        <scale>1.3</scale>
        <Icon>
        <href>http://maps.google.com/mapfiles/kml/shapes/target.png</href>
        </Icon>
    </IconStyle>
    </Style>
    
    <!-- Style for mean apogee -->
    <Style id="meanApogeeStyle">
    <IconStyle>
        <color>ffff00aa</color>
        <scale>1.3</scale>
        <Icon>
        <href>http://maps.google.com/mapfiles/kml/shapes/square.png</href>
        </Icon>
    </IconStyle>
    </Style>
    
    <!-- Style for landing std dev circles -->
    <Style id="landingSigma1">
    <LineStyle>
        <color>ff0000ff</color>
        <width>3</width>
    </LineStyle>
    <PolyStyle>
        <color>220000ff</color>
        <fill>1</fill>
        <outline>1</outline>
    </PolyStyle>
    </Style>
    
    <Style id="landingSigma2">
    <LineStyle>
        <color>ff0099ff</color>
        <width>3</width>
    </LineStyle>
    <PolyStyle>
        <color>110099ff</color>
        <fill>1</fill>
        <outline>1</outline>
    </PolyStyle>
    </Style>
    
    <Style id="landingSigma3">
    <LineStyle>
        <color>ff00ffff</color>
        <width>3</width>
    </LineStyle>
    <PolyStyle>
        <color>0800ffff</color>
        <fill>1</fill>
        <outline>1</outline>
    </PolyStyle>
    </Style>
    
    <!-- Style for apogee std dev circles -->
    <Style id="apogeeSigma1">
    <LineStyle>
        <color>ffff00ff</color>
        <width>3</width>
    </LineStyle>
    <PolyStyle>
        <color>22ff00ff</color>
        <fill>1</fill>
        <outline>1</outline>
    </PolyStyle>
    </Style>
    
    <Style id="apogeeSigma2">
    <LineStyle>
        <color>ffff99ff</color>
        <width>3</width>
    </LineStyle>
    <PolyStyle>
        <color>11ff99ff</color>
        <fill>1</fill>
        <outline>1</outline>
    </PolyStyle>
    </Style>
    
    <Style id="apogeeSigma3">
    <LineStyle>
        <color>ffffff00</color>
        <width>3</width>
    </LineStyle>
    <PolyStyle>
        <color>08ffff00</color>
        <fill>1</fill>
        <outline>1</outline>
    </PolyStyle>
    </Style>
    
    <!-- Launch site -->
    <Placemark>
    <name>Launch Site</name>
    <description>Origin point (0,0)</description>
    <styleUrl>#launchStyle</styleUrl>
    <Point>
        <coordinates>{origin_lon},{origin_lat},0</coordinates>
    </Point>
    </Placemark>
    
    <!-- Mean landing point -->
    <Placemark>
    <name>Mean Landing Point</name>
    <description>Average: {mean_x_landing:.1f}m East, {mean_y_landing:.1f}m North&lt;br/&gt;Std Dev: {std_x_landing:.1f}m x {std_y_landing:.1f}m</description>
    <styleUrl>#meanLandingStyle</styleUrl>
    <Point>
        <coordinates>{mean_lon_landing},{mean_lat_landing},0</coordinates>
    </Point>
    </Placemark>
    
    <!-- Mean apogee point -->
    <Placemark>
    <name>Mean Apogee Point</name>
    <description>Average: {mean_x_apogee:.1f}m East, {mean_y_apogee:.1f}m North, {mean_z_apogee:.1f}m altitude&lt;br/&gt;Std Dev: {std_x_apogee:.1f}m x {std_y_apogee:.1f}m (horizontal), {std_z_apogee:.1f}m (vertical)</description>
    <styleUrl>#meanApogeeStyle</styleUrl>
    <Point>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>{mean_lon_apogee},{mean_lat_apogee},{mean_z_apogee}</coordinates>
    </Point>
    </Placemark>
"""

# Add landing standard deviation ellipses
for sigma, style, name in [(1, 'landingSigma1', 'Landing 1 Std Dev'), 
                            (2, 'landingSigma2', 'Landing 2 Std Dev'), 
                            (3, 'landingSigma3', 'Landing 3 Std Dev')]:
    lons, lats = create_circle(mean_lon_landing, mean_lat_landing, sigma * std_x_landing, sigma * std_y_landing)
    coords = ' '.join([f"{lon},{lat},0" for lon, lat in zip(lons, lats)])
    
    kml_content += f"""
    <Placemark>
    <name>{name} Ellipse</name>
    <description>{name}: {sigma * std_x_landing:.1f}m x {sigma * std_y_landing:.1f}m</description>
    <styleUrl>#{style}</styleUrl>
    <Polygon>
        <outerBoundaryIs>
        <LinearRing>
            <coordinates>
            {coords}
            </coordinates>
        </LinearRing>
        </outerBoundaryIs>
    </Polygon>
    </Placemark>
"""

# Add apogee standard deviation ellipses
for sigma, style, name in [(1, 'apogeeSigma1', 'Apogee 1 Std Dev'), 
                            (2, 'apogeeSigma2', 'Apogee 2 Std Dev'), 
                            (3, 'apogeeSigma3', 'Apogee 3 Std Dev')]:
    lons, lats = create_circle(mean_lon_apogee, mean_lat_apogee, sigma * std_x_apogee, sigma * std_y_apogee)
    coords = ' '.join([f"{lon},{lat},{mean_z_apogee}" for lon, lat in zip(lons, lats)])
    
    kml_content += f"""
    <Placemark>
    <name>{name} Ellipse</name>
    <description>{name}: {sigma * std_x_apogee:.1f}m x {sigma * std_y_apogee:.1f}m at ~{mean_z_apogee:.1f}m altitude</description>
    <styleUrl>#{style}</styleUrl>
    <Polygon>
        <altitudeMode>absolute</altitudeMode>
        <outerBoundaryIs>
        <LinearRing>
            <coordinates>
            {coords}
            </coordinates>
        </LinearRing>
        </outerBoundaryIs>
    </Polygon>
    </Placemark>
"""

# Add all landing points
for i, (lon, lat, x, y) in enumerate(zip(lon_landing, lat_landing, x_data, y_data), 1):
    kml_content += f"""
    <Placemark>
    <name>Landing {i}</name>
    <description>Offset: {x:.1f}m East, {y:.1f}m North</description>
    <styleUrl>#landingStyle</styleUrl>
    <Point>
        <coordinates>{lon},{lat},0</coordinates>
    </Point>
    </Placemark>
"""

# Add all apogee points
for i, (lon, lat, x, y, z) in enumerate(zip(lon_apogee, lat_apogee, x_apogee, y_apogee, z_apogee), 1):
    kml_content += f"""
    <Placemark>
    <name>Apogee {i}</name>
    <description>Position: {x:.1f}m East, {y:.1f}m North&lt;br/&gt;Altitude: {z:.1f}m</description>
    <styleUrl>#apogeeStyle</styleUrl>
    <Point>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>{lon},{lat},{z}</coordinates>
    </Point>
    </Placemark>
"""

kml_content += """
</Document>
</kml>
"""

# Save to file with UTF-8 encoding
filename = "rocket_flight_data.kml"
with open(filename, 'w', encoding='utf-8') as f:
    f.write(kml_content)

print(f"KML file saved as '{filename}'")
print(f"\nLaunch site: {origin_lat:.6f}, {origin_lon:.6f}")
print(f"\nLanding Statistics:")
print(f"  Mean: {mean_lat_landing:.6f}, {mean_lon_landing:.6f} ({mean_x_landing:.1f}m E, {mean_y_landing:.1f}m N)")
print(f"  Std Dev: {std_x_landing:.1f}m (E-W) x {std_y_landing:.1f}m (N-S)")
print(f"\nApogee Statistics:")
print(f"  Mean: {mean_lat_apogee:.6f}, {mean_lon_apogee:.6f} ({mean_x_apogee:.1f}m E, {mean_y_apogee:.1f}m N)")
print(f"  Mean Altitude: {mean_z_apogee:.1f}m ± {std_z_apogee:.1f}m")
print(f"  Horizontal Std Dev: {std_x_apogee:.1f}m (E-W) x {std_y_apogee:.1f}m (N-S)")
print(f"\nTotal flights: {len(lat_landing)}")