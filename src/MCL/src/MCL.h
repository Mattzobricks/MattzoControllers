void levelCrossingCommand(int levelCrossingCommand);
void basculeBridgeCommand(int bridgeCommand);
void writeLevelCrossingStatusInfo();
void handleLevelCrossingSensorEvent(int triggeredSensor);
bool lcIsOccupied();
void setServoAngle(int servoIndex, int servoAngle);
void setSignalLED(int signalIndex, bool ledState);
void setLEDBySensorStates();
void handleRemoteSensorEvent(int mcId, int sensorAddress, bool sensorState);

#if USE_PCA9685
void setupPCA9685();
#endif
#if USE_MCP23017
  void setupMCP23017();
#endif
void setServoSleepMode(bool onOff);
int mapAngle2PulseLength(int angle);