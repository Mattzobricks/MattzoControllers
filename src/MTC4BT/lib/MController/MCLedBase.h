#pragma once

class MCLedBase
{
  public:
	MCLedBase(int pwmChannel, int pin, bool inverted = false);

	int GetPin();

	void SetCurrentPwrPerc(int16_t pwrPerc);

  private:
	int _pwmChannel;
	int _pin;
	bool _inverted;

	void init();
	int16_t mapPwrPercToRaw(int pwrPerc);

	// The following (derived) classes can access private members of MCLedBase.
	friend class MCLed;
	friend class MCStatusLed;
};