#ifndef ENES100_MARKER_H_
#define ENES100_MARKER_H_

#include <Arduino.h>

namespace enes100 {
class Marker {
  public:
    int id;
    float x;
    float y;
    float theta;
    float time;
    void parseMarkerStringCSV(char* marker_string) {
      char temp_buffer[20];
      int i=0, j=0;
      //Parse id
      while(marker_string[i] != ',' && j < 20)
      {
        temp_buffer[j] = marker_string[i];
        j++;
        i++;
      }
      temp_buffer[j] = 0;
      j = 0;
      i++;
      this->id = atoi(temp_buffer);
      
      //Parse X
      while(marker_string[i] != ',' && j < 20)
      {
        temp_buffer[j] = marker_string[i];
        j++;
        i++;
      }
      temp_buffer[j] = 0;
      j = 0;
      i++;
	  this->x = atof(temp_buffer);
      //Parse y
      while(marker_string[i] != ',' && j < 20)
      {
        temp_buffer[j] = marker_string[i];
        j++;
        i++;
      }
      temp_buffer[j] = 0;
      j = 0;
      i++;
	  this->y = atof(temp_buffer);
      //Parse theta
      while(marker_string[i] != ',' && j < 20)
      {
        temp_buffer[j] = marker_string[i];
        j++;
        i++;
      }
      temp_buffer[j] = 0;
      j = 0;
      i++;
      this->theta = atof(temp_buffer);
      //Parse time
      while(marker_string[i] != 0 && j < 20)
      {
        temp_buffer[j] = marker_string[i];
        j++;
        i++;
      }
      temp_buffer[j] = 0;
      this->time = atol(temp_buffer);
    };
};
}

#endif