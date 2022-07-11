#include "../../pack_fw_states/pack_fw_states.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "pack_fw3_states");

  PACK_FW_STATES _pack_fw3;
  if (true) {
    _pack_fw3.set_planeID(3);
    _pack_fw3.run(argc, argv);
  }

  return 0;
}
