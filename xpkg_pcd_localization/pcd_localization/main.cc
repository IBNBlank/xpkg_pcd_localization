/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui 847235539@qq.com
 * Date   : 2023-08-29
 ****************************************************************/

#include <ros/ros.h>

#include "pcd_localization/data_interface.h"
#include "pcd_localization/pcd_localization.h"

using hex::localization::DataInterface;
using hex::localization::LogLevel;
using hex::localization::PcdLocalization;

void TimeCallback() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static PcdLocalization& pcd_localization =
      PcdLocalization::GetPcdLocalization();
  static DataInterface& data_interface = DataInterface::GetDataInterface();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (pcd_localization.Init()) {
        data_interface.Log(LogLevel::kInfo, "%s : Init Succeded",
                           "pcd_localization");
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Init Failed",
                           "pcd_localization");
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (pcd_localization.Work()) {
        // data_interface.Log(LogLevel::kInfo, "%s : Work Succeded",
        //                    "pcd_localization");
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        data_interface.Log(LogLevel::kWarn, "%s : Work Failed",
                           "pcd_localization");
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      data_interface.Log(LogLevel::kError, "%s : Unknown State",
                         "pcd_localization");
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

int main(int argc, char* argv[]) {
  DataInterface& data_interface = DataInterface::GetDataInterface();
  data_interface.Init(argc, argv, "pcd_localization", 50.0, TimeCallback);

  data_interface.Work();

  data_interface.Deinit();

  return 0;
}
