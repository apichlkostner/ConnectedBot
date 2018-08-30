//    Robot control
//    Copyright (C) 2017 Arthur Pichlkostner <apichlkostner@gmx.de>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>

#ifndef DISTANCESENSOR_H_
#define DISTANCESENSOR_H_

#include <Arduino.h>

namespace ConnectedBotFirmware {

class DistanceSensor {
 public:
  DistanceSensor(uint8_t pin) : pin_(pin) {}

  virtual ~DistanceSensor();

  float getDistance();

 protected:
  uint8_t pin_;
};

} /* namespace ConnectedBotFirmware */

#endif /* DISTANCESENSOR_H_ */
