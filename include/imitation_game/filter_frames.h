
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class FilterFrames
{
private:
  tf::StampedTransform frame_transform_, filtered_frame_transform_;
  tf::TransformListener frame_tf_listener_;
  tf::TransformBroadcaster filtered_frame_broadcaster_;

  int size = 200;
  std::vector<std::vector<float>> rotation_vector_;
  std::vector<std::vector<float>> translation_vector_;

  bool publish_frames_ = true;
  std::vector<std::string> frame_names_;
  std::string root_frame_name_;
  std::string PREFIX_BACKSLASH = "/";
  std::string PREFIX_FILTERED = "filtered/";

  void getRotationVectorFromTransform(const std::vector<tf::StampedTransform>& transform);
  void getTranslationVectorFromTransform(const std::vector<tf::StampedTransform>& transform);
  void copyValuesIntoVectors(const tf::StampedTransform& transform, int i);

  void clearVector(std::vector<std::vector<float>>& vector);
  void resizeVector(std::vector<std::vector<float>>& vector, int elements, int size);

  void filteredFramesPublisher();
  
public:
  FilterFrames(const std::vector<std::string>& frame_names, std::string root_frame);
  FilterFrames(const std::string& frame_names, std::string root_frame);
  ~FilterFrames();

  tf::Quaternion filterRotationMedian(std::vector<std::vector<float>>& quaternion_vec);
  tf::Vector3 filterTranslationMedian(std::vector<std::vector<float>>& position_vec);
  tf::StampedTransform filterFrame(const std::vector<tf::StampedTransform>& frames_vec);

  bool getFilterPublisherStatus();
  void setFilterPublisherStatus(bool publish_filtered_frames_ = true);

  void operator()()
  {
    filteredFramesPublisher();
  }
};