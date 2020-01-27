#include <iostream>
#include <sstream>
#include <math.h>
#include <chrono>
#include <thread>
#include <ctime>
#include "cppZOEM8.h"
#include "magfield.h"

using namespace std::chrono;

#define ZOEM8Q_ADDR               0x42

const float ZOEM8::read_interval = 0.1;

ZOEM8::ZOEM8() {
	magnetic_declination = -14.42; // declination at MIT pavilion
}

bool ZOEM8::init() {
  if ((bus = i2c_open("/dev/i2c-2")) == -1) {
    std::cout << "i2c bus could not be opened... Exiting." << std::endl;
    exit(0);
  }
  device.bus = bus;
  device.addr = ZOEM8Q_ADDR;
  device.tenbit = 0;
  device.delay = 1;
  device.flags = 0;
  device.page_bytes = 8;
  device.iaddr_bytes = 0;

	return true;
}

bool ZOEM8::read() {
	response = "";
	while(1) {
		if(i2c_ioctl_read(&device, 0x0, rbuf, 1) != 1) {
			std::cout << "Read error... Exiting!" << std::endl;
			exit(0);
		} else if(rbuf[0] == 255) {
			return false;
		} else if(rbuf[0] == 10) {
			break;
		} else {
			response += char(rbuf[0]);
		}
	}
	parse_response(response);
	return true;
}

void ZOEM8::parse_response(std::string gps_line) {
  gps_components.clear();
  std::stringstream s_stream(gps_line);
  while(s_stream.good()) {
    std::string substr;
    getline(s_stream, substr, ',');
    gps_components.push_back(substr);
  }

  if(gps_components[0] == "$GNGGA") {
    std::string check_sum = gps_components[gps_components.size()-1];

    check_sum = check_sum.substr(check_sum.size()-3, check_sum.size()-2);
    int check_val = 0;
    for(unsigned int i = 1; i < gps_line.size()-4; i++) {
      int unicode = (char)gps_line[i];
      check_val = check_val^unicode;
    }
    int check_sum_int = std::stoi(check_sum,nullptr,16);

    if(check_val == check_sum_int) {
      if(gps_components[2] != "") {
        latitude = std::stod(gps_components[2]);
        unsigned int lat_d = (int)floor(latitude/100);
        double lat_m = latitude - lat_d*100;
        latitude = lat_d + lat_m/60.0;
        if(gps_components[3] == "S") latitude = -latitude;
      }
      if(gps_components[4] != "") {
        longitude = std::stod(gps_components[4]);
        unsigned int lon_d = (int)floor(longitude/100);
        double lon_m = longitude - lon_d*100;
        longitude = lon_d + lon_m/60.0;
        if(gps_components[5] == "W") longitude = -longitude;
      }
      if(gps_components[2] != "" && gps_components[4] != "") {
        magnetic_declination = get_magnetic_declination(latitude, longitude);
      }
      if(gps_components[6] != "") {
        int qual = std::stoi(gps_components[6]);
        if(qual == 0) quality = "invalid";
        else if (qual == 1) quality = "single point";
        else if (qual == 2) quality = "pseudorange differential";
        else if (qual == 4) quality = "RTK fixed";
        else if (qual == 5) quality = "RTK floating";
        else if (qual == 6) quality = "dead reckoning";
        else if (qual == 7) quality = "manual input";
        else if (qual == 8) quality = "simulator";
        else if (qual == 9) quality = "WAAS";
        else quality = "invalid";
      }
      if(gps_components[7] != "") {
        num_satellites = std::stoi(gps_components[7]);
      }
      if(gps_components[8] != "") {
        horizontal_dilution = stod(gps_components[8]);
      }
      if(gps_components[9] != "") {
        altitude = stod(gps_components[9]);
      }
      if(gps_components[11] != "") {
        geoid_undulation = stod(gps_components[11]);
      }
    }
  }

  if(gps_components[0] == "$GNRMC") {
    std::string check_sum = gps_components[gps_components.size()-1];

    check_sum = check_sum.substr(check_sum.size()-3, check_sum.size()-2);
    int check_val = 0;
    for(unsigned int i = 1; i < gps_line.size()-4; i++) {
      int unicode = (char)gps_line[i];
      check_val = check_val^unicode;
    }
    int check_sum_int = std::stoi(check_sum,nullptr,16);

    if(check_val == check_sum_int) {
      if(gps_components[1] != "" && gps_components[9] != "") {
        utc_time = stod(gps_components[1]);
        std::string date = gps_components[9];
        int hours = (int)floor(utc_time/1e4);
        int mins = (int)floor((utc_time - hours*1e4)/1e2);
        int secs = (int)floor(utc_time - (hours*1e4 + mins*1e2));
        int usecs = (int)floor((utc_time - (hours*1e4 + mins*1e2 + secs))*1e6);
        int year = (int)stoi(date.substr(4,2)) + 2000;
        int month = (int)stoi(date.substr(2,2));
        int day = (int)stoi(date.substr(0,2));
        std::tm tm;
        tm.tm_mday = day;
        tm.tm_mon = month;
        tm.tm_year = year - 1900;
        tm.tm_hour = hours;
        tm.tm_min = mins;
        tm.tm_sec = secs;
        std::time_t tt = mktime(&tm);
        utc = (long)tt + usecs;
      }
      position_status = gps_components[2];
      if(position_status == "A") position_status = "valid";
      else position_status = "invalid";
      if(gps_components[7] != "") {
        speed_over_ground = stod(gps_components[7]);
      }
      if(gps_components[8] != "") {
        course_over_ground = stod(gps_components[8]);
      }
      mode = gps_components[12];
      if(mode == "N") mode = "invalid";
      else if(mode == "A") mode = "autonomous";
      else if(mode == "D") mode = "differential";
      else if(mode == "E") mode = "dead-reckoning";
      else mode = "invalid";
    }
  }
}

double ZOEM8::get_magnetic_declination(double lat, double lon) {
	double field[6];
  double h = 0.0;
  time_t curTime = time(NULL);
  struct tm *lTime = localtime(&curTime);
  int mm = lTime->tm_mon + 1;
  int dd = lTime->tm_mday;
  int yy = lTime->tm_year + 1900;
  yy = yy - 2000; //year should only be two digits
  int model = 13; //model 13 is WMM 2020
  std::cout << "Latitude: " << lat << std::endl;
  std::cout << "Longitude: " << lon << std::endl;
  std::cout << "Year: " << yy << std::endl;
  std::cout << "Month: " << mm << std::endl;
  std::cout << "Day: " << dd << std::endl;
  double declination = rad_to_deg(SGMagVar(deg_to_rad(lat),deg_to_rad(lon),h,yymmdd_to_julian_days(yy,mm,dd),model,field));
  std::cout << "Magnetic Declination: " << declination << std::endl;
  return declination;
}

int main(int argc, char *argv[])
{
  ZOEM8 gps_device;
  gps_device.init();
	while(1) {
		gps_device.read();
	}
}
