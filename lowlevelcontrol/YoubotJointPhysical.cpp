#include "YoubotJointPhysical.hpp"
#include "TMCLMailboxMessage.hpp"
#include "Time.hpp"
#include "Logger.hpp"
#include <sstream>
#include <stdexcept>

using namespace youbot;
using namespace youbot::intrinsic;

void YoubotJointPhysical::GetFirmwareVersionViaMailbox(int& controllernum,
  int& firmwareversion) {
  auto ptr = GetFirmware::InitSharedPtr(GetSlaveIndex());
  int n_try = 0;
  do {
    SLEEP_MILLISEC(5)
    center->SendMessage_(ptr);
    ptr->GetOutput(controllernum, firmwareversion);
    n_try++;
    if (n_try>=50)
      throw std::runtime_error("Controller/firmware type zeros returned again and again");
  } while (controllernum == 0 && firmwareversion == 0);
  if (controllernum != 1610 || firmwareversion != 148) {
    log(Log::fatal, "Not supported joint with slaveindex " + std::to_string(GetSlaveIndex()) + ". Controller: "
      + std::to_string(controllernum) + " Firmware: " + std::to_string(firmwareversion));
    throw std::runtime_error("Not supported joint controller/firmware type");
  }
  else
    log(Log::info, "Joint with slaveindex " + std::to_string(GetSlaveIndex()) + " initialized. Controller: "
      + std::to_string(controllernum) + " Firmware: " + std::to_string(firmwareversion));
}

unsigned int YoubotJointPhysical::GetEncoderResolutionViaMailbox() {
  auto ptr = GetEncoderStepsPerRotation::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  int out = ptr->GetReplyValue();
  log(Log::info, "Init joint " + std::to_string(GetSlaveIndex()) +
    " GetEncoderStepsPerRotation: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return out;
}

YoubotJointPhysical::YoubotJointPhysical(int slaveIndex, const std::map<std::string,
  double>& config, EtherCATMaster::Ptr center)
    : YoubotJoint(slaveIndex, config, center) {
  // Check if there is sg like the motor driver on the ethercat bus
  if (slaveIndex >= center->getSlaveNum())
    throw std::runtime_error("Slave index " + std::to_string(slaveIndex) + " not found");
  if (center->getSlaveName(slaveIndex).compare("TMCM-1610") != 0)
    throw std::runtime_error("Unknown module name " + center->getSlaveName(slaveIndex));
  // Set buffer size
  center->SetProcessFromSlaveSize(20, slaveIndex);
}

void YoubotJointPhysical::ConfigControlParameters(bool forceConfiguration) {
  if (!forceConfiguration && IsConfiguratedViaMailbox()) {
    log(Log::info, "Joint " + std::to_string(GetSlaveIndex()) + " is already configurated");
    return;
  }
  // 1. Stop the motor
  StopViaMailbox();
  // SetMaxRampVelocity
  {
    auto ptr = SetMaxRampVelocityRPM::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("MaximumPositioningVelocityMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetMaxRampVelocityRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetAccelerationParam
  {
    auto ptr = SetAccelerationParamRPMPSec::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("MotorAccelerationMotorRPMPerSec")));
    center->SendMessage_(ptr);
    log(Log::info, " SetAccelerationParamRPMPSec: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PositionControlSwitchingThresholdMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetTresholdSpeedForPosPIDRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("SpeedControlSwitchingThresholdMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetTresholdSpeedForVelPIDRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP1ParameterPositionControl
  {
    auto ptr = SetP1ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI1ParameterPositionControl
  {
    auto ptr = SetI1ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD1ParameterPositionControl
  {
    auto ptr = SetD1ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("DParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping1ParameterPositionControl
  {
    auto ptr = SetClipping1ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IClippingParameterFirstParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP1ParameterVelocityControl
  {
    auto ptr = SetP1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD1ParameterVelocityControl
  {
    auto ptr = SetD1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("DParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI1ParameterVelocityControl
  {
    auto ptr = SetI1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping1ParameterVelocityControl
  {
    auto ptr = SetClipping1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IClippingParameterFirstParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP2ParameterPositionControl
  {
    auto ptr = SetP2ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI2ParameterPositionControl
  {
    auto ptr = SetI2ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD2ParameterPositionControl
  {
    auto ptr = SetD2ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("DParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping2ParameterPositionControl
  {
    auto ptr = SetClipping2ParameterPositionControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IClippingParameterSecondParametersPositionControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP2ParameterVelocityControl
  {
    auto ptr = SetP2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI2ParameterVelocityControl
  {
    auto ptr = SetI2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD2ParameterVelocityControl
  {
    auto ptr = SetD2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("DParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping2ParameterVelocityControl
  {
    auto ptr = SetClipping2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IClippingParameterSecondParametersSpeedControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetP2ParameterCurrentControl
  {
    auto ptr = SetP2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetP2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetI2ParameterCurrentControl
  {
    auto ptr = SetI2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetI2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetD2ParameterCurrentControl
  {
    auto ptr = SetD2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("DParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetD2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetClipping2ParameterCurrentControl
  {
    auto ptr = SetClipping2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("IClippingParameterCurrentControl")));
    center->SendMessage_(ptr);
    log(Log::info, " SetClipping2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetMaxVelocityToReachTargetRPM
  {
    auto ptr = SetMaxVelocityToReachTargetRPM::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("MaximumVelocityToSetPositionMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetMaxVelocityToReachTargetRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetMaxDistanceToReachTarget
  {
    auto ptr = SetMaxDistanceToReachTarget::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("PositionTargetReachedDistance")));
    center->SendMessage_(ptr);
    log(Log::info, " SetMaxDistanceToReachTarget: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  // SetVelThresholdHallFXRPM
  {
    auto ptr = SetVelThresholdHallFXRPM::InitSharedPtr(GetSlaveIndex(),
      int32_t(GetConfig().at("VelocityThresholdForHallFXMotorRPM")));
    center->SendMessage_(ptr);
    log(Log::info, " SetVelThresholdHallFXRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  }
  SetConfiguratedViaMailbox();
}

bool YoubotJointPhysical::CheckControlParameters() {
  // GetMaxRampVelocityRPM
  {
    auto ptr = GetMaxRampVelocityRPM::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("MaximumPositioningVelocityMotorRPM"));
    log(Log::info, " GetMaxRampVelocityRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetAccelerationParamRPMPSec
  {
    auto ptr = GetAccelerationParamRPMPSec::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("MotorAccelerationMotorRPMPerSec"));
    log(Log::info, " GetAccelerationParamRPMPSec: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForPosPIDRPM
  {
    auto ptr = GetTresholdSpeedForPosPIDRPM::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PositionControlSwitchingThresholdMotorRPM"));
    log(Log::info, " GetTresholdSpeedForPosPIDRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetTresholdSpeedForVelPIDRPM
  {
    auto ptr = GetTresholdSpeedForVelPIDRPM::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("SpeedControlSwitchingThresholdMotorRPM"));
    log(Log::info, " GetTresholdSpeedForVelPIDRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP1ParameterPositionControl
  {
    auto ptr = GetP1ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PParameterFirstParametersPositionControl"));
    log(Log::info, " GetP1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI1ParameterPositionControl
  {
    auto ptr = GetI1ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IParameterFirstParametersPositionControl"));
    log(Log::info, " GetI1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD1ParameterPositionControl
  {
    auto ptr = GetD1ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("DParameterFirstParametersPositionControl"));
    log(Log::info, " GetD1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping1ParameterPositionControl
  {
    auto ptr = GetClipping1ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IClippingParameterFirstParametersPositionControl"));
    log(Log::info, " GetClipping1ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP1ParameterVelocityControl
  {
    auto ptr = GetP1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PParameterFirstParametersSpeedControl"));
    log(Log::info, " GetP1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD1ParameterVelocityControl
  {
    auto ptr = GetD1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("DParameterFirstParametersSpeedControl"));
    log(Log::info, " GetD1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI1ParameterVelocityControl
  {
    auto ptr = GetI1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IParameterFirstParametersSpeedControl"));
    log(Log::info, " GetI1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping1ParameterVelocityControl
  {
    auto ptr = GetClipping1ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IClippingParameterFirstParametersSpeedControl"));
    log(Log::info, " GetClipping1ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterPositionControl
  {
    auto ptr = GetP2ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PParameterSecondParametersPositionControl"));
    log(Log::info, " GetP2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterPositionControl
  {
    auto ptr = GetI2ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IParameterSecondParametersPositionControl"));
    log(Log::info, " GetI2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterPositionControl
  {
    auto ptr = GetD2ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("DParameterSecondParametersPositionControl"));
    log(Log::info, " GetD2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterPositionControl
  {
    auto ptr = GetClipping2ParameterPositionControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IClippingParameterSecondParametersPositionControl"));
    log(Log::info, " GetClipping2ParameterPositionControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterVelocityControl
  {
    auto ptr = GetP2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PParameterSecondParametersSpeedControl"));
    log(Log::info, " GetP2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterVelocityControl
  {
    auto ptr = GetI2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IParameterSecondParametersSpeedControl"));
    log(Log::info, " GetI2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterVelocityControl
  {
    auto ptr = GetD2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("DParameterSecondParametersSpeedControl"));
    log(Log::info, " GetD2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterVelocityControl
  {
    auto ptr = GetClipping2ParameterVelocityControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IClippingParameterSecondParametersSpeedControl"));
    log(Log::info, " GetClipping2ParameterVelocityControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetP2ParameterCurrentControl
  {
    auto ptr = GetP2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PParameterCurrentControl"));
    log(Log::info, " GetP2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetI2ParameterCurrentControl
  {
    auto ptr = GetI2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IParameterCurrentControl"));
    log(Log::info, " GetI2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetD2ParameterCurrentControl
  {
    auto ptr = GetD2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("DParameterCurrentControl"));
    log(Log::info, " GetD2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetClipping2ParameterCurrentControl
  {
    auto ptr = GetClipping2ParameterCurrentControl::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("IClippingParameterCurrentControl"));
    log(Log::info, " GetClipping2ParameterCurrentControl: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetMaxVelocityToReachTargetRPM
  {
    auto ptr = GetMaxVelocityToReachTargetRPM::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("MaximumVelocityToSetPositionMotorRPM"));
    log(Log::info, " GetMaxVelocityToReachTargetRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetMaxDistanceToReachTarget
  {
    auto ptr = GetMaxDistanceToReachTarget::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("PositionTargetReachedDistance"));
    log(Log::info, " GetMaxDistanceToReachTarget: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  // GetVelThresholdHallFXRPM
  {
    auto ptr = GetVelThresholdHallFXRPM::InitSharedPtr(GetSlaveIndex());
    center->SendMessage_(ptr);
    int32_t fromconfig = int32_t(GetConfig().at("VelocityThresholdForHallFXMotorRPM"));
    log(Log::info, " GetVelThresholdHallFXRPM: " + std::to_string(ptr->GetReplyValue()) + "(=" + std::to_string(fromconfig) + ")" + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
    if (ptr->GetReplyValue() != fromconfig)
      return false;
  }
  return true;
}

void YoubotJointPhysical::RotateJointRightViaMailbox(double speedJointRadPerSec) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(GetSlaveIndex(),
    int32_t(speedJointRadPerSec / GetParameters().gearRatio / 2. / M_PI * 60.));
  center->SendMessage_(ptr);
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJointPhysical::RotateJointLeftViaMailbox(double speedJointRadPerSec) {
  auto ptr = RotateLeftMotorRPM::InitSharedPtr(GetSlaveIndex(),
    int32_t(speedJointRadPerSec / GetParameters().gearRatio / 2. / M_PI * 60.));
  center->SendMessage_(ptr);
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJointPhysical::StopViaMailbox() {
  auto ptr = MotorStop::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " MotorStop: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

YoubotJointPhysical::JointStatus YoubotJointPhysical::GetJointStatusViaMailbox() {
  auto ptr = GetErrorStatusFlag::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, "GetErrorStatusFlag: " +
    TMCL::StatusErrorFlagsToString(ptr->GetReplyValue())
    + "(" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return ptr->GetReplyValue();
}

void YoubotJointPhysical::ResetTimeoutViaMailbox() {
  auto ptr = ClearMotorControllerTimeoutFlag::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, "  ClearMotorControllerTimeoutFlag: " + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()));
}

void YoubotJointPhysical::ResetI2TExceededViaMailbox() {
  auto ptr = ClearI2TFlag::InitSharedPtr(GetSlaveIndex());
  log(Log::info, "  ClearI2TFlag: waiting");
  SLEEP_MILLISEC(long(GetParameters().cooldowntime_sec * 1000))
  center->SendMessage_(ptr);
  log(Log::info, "  ClearI2TFlag: " + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + " waiting done");
}

void YoubotJointPhysical::StartInitializationViaMailbox() {
  StopViaMailbox();
  auto status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());
  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();
  if (status.Timeout())
    ResetTimeoutViaMailbox();
  status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());
  auto ptr = SetInitialize::InitSharedPtr(GetSlaveIndex(), 1);
  center->SendMessage_(ptr);
  log(Log::info, "  SetInitialize: " + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()));
}

bool YoubotJointPhysical::IsInitializedViaMailbox() {
  auto ptr = GetInitialized::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, "  GetInitialized: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return ptr->GetReplyValue();
}

bool YoubotJointPhysical::IsConfiguratedViaMailbox() {
  auto ptr = GetNeedConfiguration::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetNeedConfiguration: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return !ptr->GetReplyValue();
}

void YoubotJointPhysical::SetConfiguratedViaMailbox() {
  auto ptr = SetIsConfigurated::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " SetIsConfigurated: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

int32_t _toInt32(uint8_t* buff) {
  return buff[3] << 24 | buff[2] << 16 | buff[1] << 8 | buff[0];
}

const YoubotJointPhysical::ProcessReturn& YoubotJointPhysical::GetProcessReturnData() {
  static ProcessBuffer buffer;
  center->GetProcessMsg(buffer, GetSlaveIndex());
  processReturn.encoderPosition = _toInt32(&buffer.buffer[0]);
  processReturn.currentmA = _toInt32(&buffer.buffer[4]);
  processReturn.motorVelocityRPM = _toInt32(&buffer.buffer[8]);
  processReturn.status = _toInt32(&buffer.buffer[12]);
  processReturn.motorPWM = _toInt32(&buffer.buffer[16]);
  processReturn.qRad = Ticks2qRad(processReturn.encoderPosition);
  processReturn.dqRadPerSec = RPM2qRadPerSec(processReturn.motorVelocityRPM);
  processReturn.tau = mA2Nm(processReturn.currentmA);
  return processReturn;
}

void YoubotJointPhysical::ReqNoAction() {
  static ProcessBuffer toSet(5);
  toSet.buffer[4] = TMCL::ControllerMode::NO_MORE_ACTION;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void YoubotJointPhysical::ReqMotorPositionTick(int ticks) {
  if (GetParameters().firmwareversion == 148) {
    log(Log::fatal, "Position control is forbidden for TMCM1610 with firmware 1.48 because of random encoder error!");
    SLEEP_MILLISEC(200);
    throw std::runtime_error("Position control is forbidden for TMCM1610 with firmware 1.48 because of random encoder error!");
  }
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = ticks >> 24;
  toSet.buffer[2] = ticks >> 16;
  toSet.buffer[1] = ticks >> 8;
  toSet.buffer[0] = ticks & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::POSITION_CONTROL;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void YoubotJointPhysical::ReqMotorSpeedRPM(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::VELOCITY_CONTROL;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void YoubotJointPhysical::ReqEncoderReference(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::SET_POSITION_TO_REFERENCE;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void YoubotJointPhysical::ReqStop() {
  static ProcessBuffer toSet(5);
  for (int i = 0; i < 4; i++)
    toSet.buffer[i] = 0;
  toSet.buffer[4] = TMCL::ControllerMode::MOTOR_STOP;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void YoubotJointPhysical::ReqVoltagePWM(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::PWM_MODE;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void youbot::YoubotJointPhysical::ReqMotorCurrentmA(int32_t value) {
  static ProcessBuffer toSet(5);
  toSet.buffer[3] = value >> 24;
  toSet.buffer[2] = value >> 16;
  toSet.buffer[1] = value >> 8;
  toSet.buffer[0] = value & 0xff;
  toSet.buffer[4] = TMCL::ControllerMode::CURRENT_MODE;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void YoubotJointPhysical::ReqInitializationViaProcess() {
  static ProcessBuffer toSet(5);
  for (int i = 0; i < 4; i++)
    toSet.buffer[i] = 0;
  toSet.buffer[4] = TMCL::ControllerMode::INITIALIZE;
  center->SetProcessMsg(toSet, GetSlaveIndex());
}

void youbot::YoubotJointPhysical::CheckI2tAndTimeoutError(JointStatus status) {
  if (status.I2TExceeded()) {
    log(Log::fatal, "I2t exceeded in slave " + std::to_string(GetSlaveIndex()) + " (" + status.toString() + ")");
    SLEEP_MILLISEC(10);
    throw std::runtime_error("I2t exceeded");
  }
  if (status.Timeout()) {
    log(Log::fatal, "Timeout in slave " + std::to_string(GetSlaveIndex()) + " (" + status.toString() + ")");
    SLEEP_MILLISEC(10);
    throw std::runtime_error("Timeout");
  }
}

double YoubotJointPhysical::GetCurrentAViaMailbox() {
  auto ptr = GetCurrent::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetCurrent[mA]: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return double(ptr->GetReplyValue()) / 1000.;
}

void YoubotJointPhysical::RotateMotorRightViaMailbox(int32_t speedMotorRPM) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(GetSlaveIndex(), speedMotorRPM);
  center->SendMessage_(ptr);
  log(Log::info, " RotateRightMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

void YoubotJointPhysical::RotateMotorLeftViaMailbox(int32_t speedMotorRPM) {
  auto ptr = RotateLeftMotorRPM::InitSharedPtr(GetSlaveIndex(), speedMotorRPM);
  center->SendMessage_(ptr);
  log(Log::info, " RotateLeftMotorRPM: " + std::to_string(ptr->GetReplyValue()) +
    " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

double YoubotJointPhysical::GetJointVelocityRadPerSecViaMailbox() {
  auto ptr = GetActualSpeedMotorRPM::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetActualSpeedMotorRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return double(ptr->GetReplyValue()) * 2. * M_PI / 60. * GetParameters().gearRatio;
}

long youbot::YoubotJointPhysical::GetI2tLimitValueViaMailbox() {
  auto ptr = GetI2tLimitValue::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetI2tLimitValue: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return ptr->GetReplyValue();
}

long youbot::YoubotJointPhysical::GetCurrentI2tValueViaMailbox() {
  auto ptr = GetCurrentI2tValue::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetCurrentI2tValue: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return ptr->GetReplyValue();
}

void YoubotJointPhysical::SetJointVelocityRadPerSecViaMailbox(double value) {
  auto ptr = RotateRightMotorRPM::InitSharedPtr(GetSlaveIndex(), value / GetParameters().gearRatio * 60. / M_PI / 2.);
  center->SendMessage_(ptr);
  log(Log::info, " GetActualSpeedMotorRPM: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

double YoubotJointPhysical::GetThermalWindingTimeSecViaMailbox() {
  auto ptr = GetThermalWindingTimeMs::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetThermalWindingTimeMs: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return double(ptr->GetReplyValue()) / 1000.;
}

void YoubotJointPhysical::SetTargetCurrentAViaMailbox(double current) {
  auto ptr = SetTargetCurrentmA::InitSharedPtr(GetSlaveIndex(), int32_t(current * 1000.));
  center->SendMessage_(ptr);
  log(Log::info, " SetTargetCurrentmA[mA]: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}

bool YoubotJointPhysical::IsCalibratedViaMailbox() {
  auto ptr = GetNeedCalibration::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " GetNeedCalibration: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
  return !ptr->GetReplyValue();
}

void YoubotJointPhysical::I2tResetTest() {
  auto status = GetJointStatusViaMailbox();
  log(Log::info, status.toString());
  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();
  bool forward = GetConfig().at("CalibrationDirection");
  const double calJointRadPerSec = 0.2;
  
  int limit = GetI2tLimitValueViaMailbox();
  
  ResetTimeoutViaMailbox();
  SetJointVelocityRadPerSecViaMailbox(forward ? calJointRadPerSec : -calJointRadPerSec);
  do
  {
    auto current = GetCurrentI2tValueViaMailbox();
    status = GetJointStatusViaMailbox();
    auto str = status.toString();
  } while (!status.I2TExceeded());

  if (status.I2TExceeded())
    ResetI2TExceededViaMailbox();

  status = GetJointStatusViaMailbox();
  
  if (status.I2TExceeded())
    log(Log::error, "Para...");
  else
    log(Log::info, "OK...");
  SLEEP_SEC(1)
}

void YoubotJointPhysical::SetCalibratedViaMailbox() {
  auto ptr = SetIsCalibrated::InitSharedPtr(GetSlaveIndex());
  center->SendMessage_(ptr);
  log(Log::info, " SetIsCalibrated: " + std::to_string(ptr->GetReplyValue()) + " (" + TMCL::RecvStatusToString(ptr->GetRecStatusFlag()) + ")");
}