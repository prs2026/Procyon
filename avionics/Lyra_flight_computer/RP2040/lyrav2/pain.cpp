String datatobelogged = "10101," + String(uptimemillis) + "," + String(missiontimemillis) + "," + String(state) + "," + 
    String(data.adjaccel_x*decimals) + "," + String(data.adjaccel_y*decimals) + "," + String(data.adjaccel_z*decimals) + "," + 
    String(data.gyro_x*decimals) + "," + String(data.gyro_y*decimals) + "," + String(data.gyro_z*decimals) + "," + 
    String(data.pitch*decimals) + "," + String(data.yaw*decimals) + "," + String(data.roll*decimals) + "," + 
    String(data.baro_alt*decimals) + "," + String(data.baro_pressure*decimals) + "," + String(data.baro_temp*decimals) + "," + 
    String(data.imu_temp*decimals) + "," + String(commandbuf*decimals) + "," + 
    String(data.rawaccel_x*decimals) + "," + String(data.rawaccel_y*decimals) + "," + String(data.rawaccel_z*decimals) + "," +
    String(data.rawpitch*decimals) + "," + String(data.rawyaw*decimals) + "," + String(data.rawroll*decimals) + "," + 
    String(battvoltage*decimals)+ "," + 
    String(data.adjabsaccel*decimals) + "," + String(data.rawabsaccel*decimals) + "," + String(data.vertical_vel*decimals) + 
    "," + String(averagerawabsaccel*decimals) + "," + String(averageverticalver*decimals) +  ","+"20202"
    ;
  