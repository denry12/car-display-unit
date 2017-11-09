

#define CARDATASCREEN_DATASCREEN		0
#define CARDATASCREEN_GEARSCREEN		1
#define CARDATASCREEN_SPEEDSCREEN		2
#define CARDATASCREEN_SHIFTLIGHTSCREEN	3




typedef struct
{
	uint16_t RPMcounter; // in amount of timer ticks
	uint16_t RPMvalue; // in RPM
	// rpm ticks multiplier or something
	uint16_t VSScounter; // in amount of timer ticks
	uint16_t VSSvalue; // in km/h
	// vss ticks multiplier or something
	uint16_t batteryValue; // in milliVolt
	uint32_t rpmVssRatio; // 32-bit to allow for calculation
	uint16_t fuelVoltage; // in milliVolt
	uint16_t fuelDetsilitre; // 60.1L displayed as "601"
	uint16_t tempVoltage; // in milliVolt
	uint16_t tempCentigrade; // no comma, 60.1C displayed as "60"

} carData_datastruct;

