# Constants for the MultiWii Serial Protocol (MSP) v1.0

MSP_SET_RAW_RC = 200        #in message          8 rc chan
MSP_SET_RAW_GPS = 201       #in message          fix, numsat, lat, lon, alt, speed

MSP_SET_PID = 202           #in message          P I D coeff (9 are used currently)

MSP_SET_RC_TUNING = 204     #in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID

MSP_SET_MAX_ALT = 218       #in message          max altitude

MSP_SET_COMMAND = 217       #in message          8 aux chan

MSP_ACC_TRIM = 240          #out message         get acc angle trim values
MSP_SET_ACC_TRIM = 239      #in message          set acc angle trim values


MSP_ATTITUDE = 108          #out message         2 angles 1 heading
MSP_ALTITUDE = 109          #out message         altitude, variometer
MSP_PID = 112               #out message         P I D coeff (9 are used currently)

MSP_RAW_IMU = 102           #out message         9 DOF


MSP_ANALOG=110


L = 1000
C = 1500
H = 2000