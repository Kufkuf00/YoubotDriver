#include "TMCLMailboxMessage.hpp"

GetFirmware::GetFirmware(int slaveIndex) : MailboxMessage(slaveIndex) {
  uint8_t* buff = getToSlaveBuff();
  *buff = (uint8_t)TMCL::Module::DRIVE;
  *(buff + 1) = (uint8_t)TMCL::Cmd::FIRMWARE_VERSION;
  *(buff + 2) = 0;
  *(buff + 3) = 0;
}

void GetFirmware::GetOutput(long& controllernum, long& firmwarenum) const {
  char* ptr, * ptr2;
  controllernum = strtol((const char*)getFromSlaveBuff(), &ptr, 10);
  firmwarenum = strtol(ptr + 1, &ptr2, 10);
}

std::shared_ptr<GetFirmware> GetFirmware::InitSharedPtr(int slaveIndex) {
  return std::make_shared<GetFirmware>(slaveIndex);
}