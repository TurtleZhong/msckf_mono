#ifndef MSCKF_VIO_NODELET_H
#define MSCKF_VIO_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <msckf_mono/msckf_mono.h>

namespace msckf_mono {
class MsckfVioNodelet : public nodelet::Nodelet {
public:
    MsckfVioNodelet() { return; }
    ~MsckfVioNodelet() { return; }

private:
    virtual void onInit();
    MsckfVioPtr msckf_vio_ptr;
};
} // end namespace msckf_mono
#endif // MSCKF_VIO_NODELET_H
