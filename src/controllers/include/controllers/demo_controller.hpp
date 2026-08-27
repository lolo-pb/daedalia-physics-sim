#pragma once

#include "controllers/controller_io.hpp"

class DemoController {
public:
  void Update(const ControllerInput &input, TargetDrone &drone);

private:
  bool armed_ = false;
  bool x_was_down_ = false;
};
