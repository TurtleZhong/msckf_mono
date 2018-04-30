#include <msckf_mono/msckf_mono_nodelet.h>

namespace msckf_mono {
void MsckfVioNodelet::onInit() {
  msckf_vio_ptr.reset(new MsckfVio(getPrivateNodeHandle()));
  if (!msckf_vio_ptr->initialize()) {
    ROS_ERROR("Cannot initialize MSCKF VIO...");
    return;
  }
  return;
}

PLUGINLIB_DECLARE_CLASS(msckf_vio, MsckfVioNodelet,
    msckf_mono::MsckfVioNodelet, nodelet::Nodelet);

}
