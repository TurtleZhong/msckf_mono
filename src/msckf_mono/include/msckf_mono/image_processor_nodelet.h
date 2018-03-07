#ifndef IMAGE_PROCESSOR_NODELET_H
#define IMAGE_PROCESSOR_NODELET_H
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <msckf_mono/image_processor.h>

namespace msckf_mono
{
class ImageProcessorNodelet : public nodelet::Nodelet
{
public:
  ImageProcessorNodelet() { return; }
  ~ImageProcessorNodelet() { return; }

private:
  virtual void onInit();
  ImageProcessorPtr img_processor_ptr;
};
} // end namespace msckf_vio

#endif // IMAGE_PROCESSOR_NODELET_H
