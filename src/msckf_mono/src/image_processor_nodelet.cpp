#include <msckf_mono/image_processor_nodelet.h>

namespace msckf_mono
{
void ImageProcessorNodelet::onInit() {
  img_processor_ptr.reset(new ImageProcessor(getPrivateNodeHandle()));
  if (!img_processor_ptr->initialize()) {
    ROS_ERROR("Cannot initialize Image Processor...");
    return;
  }
  return;
}

PLUGINLIB_DECLARE_CLASS(msckf_mono, ImageProcessorNodelet,
    msckf_mono::ImageProcessorNodelet, nodelet::Nodelet);

} // end namespace msckf_mono
