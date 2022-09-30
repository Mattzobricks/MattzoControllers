void levelCrossingCommand(int levelCrossingCommand);
void basculeBridgeCommand(int bridgeCommand);
void writeLevelCrossingStatusInfo();
void handleLevelCrossingSensorEvent(int triggeredSensor);
bool lcIsOccupied();
void setServoAngle(int servoIndex, int servoAngle);
void setSignalLED(int signalIndex, bool ledState);
void setLEDBySensorStates();
void handleRemoteSensorEvent(int mcId, int sensorAddress, bool sensorState);