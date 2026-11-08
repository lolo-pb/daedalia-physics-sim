#pragma once

#include "controllers/controller_io.hpp"

class DemoController {
public:
  void Reset();
  void Update(const ControllerInput &input, ControllerOutput &output);

private:
  bool armed_ = false;
  bool x_was_down_ = false;
};
