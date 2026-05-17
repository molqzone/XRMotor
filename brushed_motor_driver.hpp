#pragma once

#include <cstdint>

#include "libxr_def.hpp"

namespace XRMotor
{

class BrushedMotorDriver
{
 public:
  virtual ~BrushedMotorDriver() = default;

  virtual LibXR::ErrorCode Init(uint32_t pwm_frequency_hz) = 0;
  virtual LibXR::ErrorCode SetOutput(float output) = 0;
  virtual LibXR::ErrorCode Coast() = 0;
  virtual LibXR::ErrorCode Brake() = 0;
  virtual LibXR::ErrorCode Stop() = 0;
};

}  // namespace XRMotor
