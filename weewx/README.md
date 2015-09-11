To install, simply copy the three python files to the BIN_ROOT/user subdirectory of your weewx installation (see www.weewx.com/docs/usersguide.htm heading "Where to find things" for the location of this directory on your system). Update your weewx.conf to include the following sections and options. Note that you can only have one driver per weewx.conf file, but you can create multiple weewx.conf files and run multiple instances of weewx on a single machine.

```
# For VP2 ISS
[Station]
    station_type = CCWXRXVP2
[CCWXRXVP2]
    ccwxrx_splitter_hostname = 127.0.0.1
    ccwxrx_splitter_port = 5772
    # Set this to the ID set on the DIP switch in the station.
    transmitter_id = 1
    driver = user.ccwxrxvp2

# For LSMS
[Station]
    station_type = CCWXRXLSMS
[CCWXRXLSMS]
    ccwxrx_splitter_hostname = 127.0.0.1
    ccwxrx_splitter_port = 5772
    # Set this to the ID set on the DIP switch in the station.
    transmitter_id = 1
    # If a soil temperature probe is installed in a port, the temperature from
    # that port will be used when calculating the soil moisture for a moisture
    # sensor installed in the corresponding port (same port number), otherwise,
    # the following default temperature will be used.
    default_soil_temp = 24 # Degrees Celcius
    # The driver will automatically detect which sensor ports are populated.
    # Uncomment and use the following two options if you would like to tell the
    # driver to ignore a sensor for some reason.
    # moisture_sensor_blacklist = '1 2 3'
    # temp_sensor_blacklist = '2 4'
    driver = user.ccwxrxlsms    
```
