#pragma once

/*
 * @file 746FTL.h
 * @brief This file contains the (dummy) classes used to communicate with
 * FlashSim
 * @author Saksham Jain (sakshamj)
 * @author (Tweaked) Ankush Jain (ankushj)
 * @bug No known bug
 */

#include "common.h"

void SendReqToFlashSim(IPC_Format *tx_msg, IPC_Format *rx_msg);
/*
 * class FTLConf - Use this class to get configuration of flash
 *
 * This class gathers configuration information by asking FlashSim (via IPC)
 */

class FTLConf : public ConfBase {
 public:
  FTLConf() {}

  ~FTLConf() {}

  /* Returns the number of packages in flash */
  size_t GetSSDSize(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_SSD_SIZE);
  }

  /* Returns the number of dies in flash */
  size_t GetPackageSize(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_PACKAGE_SIZE);
  }

  /* Returns the number of planes in flash */
  size_t GetDieSize(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_DIE_SIZE);
  }

  /* Returns the number of blocks in flash */
  size_t GetPlaneSize(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_PLANE_SIZE);
  }

  /* Returns the number of pages in flash */
  size_t GetBlockSize(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_BLOCK_SIZE);
  }

  /* Returns the block lifetime of flash */
  size_t GetBlockEraseCount(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_BLOCK_ERASES);
  }

  /* Returns the overprovisioning (as percentage) of flash */
  size_t GetOverprovisioning(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_OVERPROVISIONING);
  }

  /* Returns the garbage collection policy of flash */
  size_t GetGCPolicy(void) const {
    return SendConfReqToFlashSim(MSG_CONF_REQ_GCPOLICY);
  }

 private:
  size_t SendConfReqToFlashSim(enum message_type_t type) const {
    IPC_Format tx_msg, rx_msg;

    tx_msg.owner_ = OWNER_FTL;
    tx_msg.type_ = type;

    SendReqToFlashSim(&tx_msg, &rx_msg);

    return rx_msg.conf_resp_;
  }

  /*
   * It is preferred not to call this function but use other functions.
   * This function is mostly for backward compatibility
   */
  int GetInteger(const std::string &key) const {
    if (key.compare(CONF_S_SSD_SIZE) == 0)
      return GetSSDSize();
    else if (key.compare(CONF_S_PACKAGE_SIZE) == 0)
      return GetPackageSize();
    else if (key.compare(CONF_S_DIE_SIZE) == 0)
      return GetDieSize();
    else if (key.compare(CONF_S_PLANE_SIZE) == 0)
      return GetPlaneSize();
    else if (key.compare(CONF_S_BLOCK_SIZE) == 0)
      return GetBlockSize();
    else if (key.compare(CONF_S_BLOCK_ERASES) == 0)
      return GetBlockEraseCount();
    else if (key.compare(CONF_S_OVERPROVISIONING) == 0)
      return GetOverprovisioning();
    else if (key.compare(CONF_S_GCPOLICY) == 0)
      return GetGCPolicy();
    else
      assert(0 && "Unknown configuration parameter");

    return -1;
  }
  /*
   * It is preferred not to call this function but use other functions.
   * This function is mostly for backward compatibility
   * FIXME:There is memory leak in this function. Try avoid this function.
   */
  const std::string &GetString(const std::string &key) const {
    std::string s;
    std::string *news_p;

    s = std::to_string(GetInteger(key));

    news_p = new std::string(s);
    return *news_p;
  }
};

/*
 * class FTLExecCallBack() - Proxy class for controller to let FTL call
 *                        its function without exposing controller internals
 *                        to the FTL
 *
 * This is the FTL side of class in IPC
 */

class FTLExecCallBack : public ExecCallBack<TEST_PAGE_TYPE> {
 public:
  /*
   * Constructor
   */
  FTLExecCallBack() {}

  /*
   * Destructor
   */
  ~FTLExecCallBack() {}

  /*
   * operator() - Mimics the behavior of a function call that calls
   *              ExecuteCommand() of class Controller
   */
  virtual void operator()(OpCode operation, Address addr) const {
    IPC_Format tx_msg, rx_msg;

    tx_msg.owner_ = OWNER_FTL;

    switch (operation) {
      case OpCode::READ:
        tx_msg.type_ = MSG_SIM_REQ_READ;
        tx_msg.sim_req_opcode_ = OpCode::READ;
        break;
      case OpCode::WRITE:
        tx_msg.type_ = MSG_SIM_REQ_WRITE;
        tx_msg.sim_req_opcode_ = OpCode::WRITE;
        break;
      case OpCode::ERASE:
        tx_msg.type_ = MSG_SIM_REQ_ERASE;
        tx_msg.sim_req_opcode_ = OpCode::ERASE;
        break;
      default:
        assert(0 && "Unknown operation");
        break;
    }

    tx_msg.sim_req_addr_ = addr;

    SendReqToFlashSim(&tx_msg, &rx_msg);
    /* Since the respnse will be empty message, rx is unimportant */
  }
};