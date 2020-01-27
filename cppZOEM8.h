#include <cstdint>
#include <vector>
#include "i2c.h"

#ifndef ZOEM8_H
#define ZOEM8_H

class ZOEM8 {
public:
	static const float read_interval;

	ZOEM8();

	bool init();

	bool read();

	void parse_response(std::string gps_line);

	double get_magnetic_declination(double lat, double lon);

private:
  int bus;
  I2CDevice device;
	unsigned char buf[10];
	unsigned char rbuf[10];
	std::string response;
	std::vector<std::string> gps_components;

	int utc_date;
	double utc_time;
	double utc;
	std::string position_status;
	std::string mode;
	double latitude;
	double longitude;
	std::string quality;
	unsigned int num_satellites;
	double horizontal_dilution;
	double altitude;
	double geoid_undulation;
	double speed_over_ground;
	double course_over_ground;
	double magnetic_declination;
};

#endif
