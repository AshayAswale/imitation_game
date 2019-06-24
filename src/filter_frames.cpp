#include <imitation_game/filter_frames.h>

FilterFrames::FilterFrames(const std::vector<std::string>& frame_names, std::string root_frame) : frame_names_(frame_names), root_frame_name_(root_frame)
{
  resizeVector(rotation_vector_, 4, size);
  resizeVector(translation_vector_, 3, size);
  ROS_INFO_STREAM("rot: " << rotation_vector_.size() << "x" << rotation_vector_.at(0).size());
  ROS_INFO_STREAM("trn: " << translation_vector_.size() << "x" << translation_vector_.at(0).size());
  frame_tf_listener_.waitForTransform(PREFIX_BACKSLASH + frame_names_.at(0), PREFIX_BACKSLASH + root_frame_name_,
                                      ros::Time::now(), ros::Duration(1.0));
}

FilterFrames::FilterFrames(const std::string& frame_names, std::string root_frame)
  : frame_names_(std::vector<std::string>{ frame_names }), root_frame_name_(root_frame)
{
  resizeVector(rotation_vector_, 4, size);
  resizeVector(translation_vector_, 3, size);
}

FilterFrames::~FilterFrames()
{
}

tf::StampedTransform FilterFrames::filterFrame(const std::vector<tf::StampedTransform>& frames_vec)
{
}

void FilterFrames::filteredFramesPublisher()
{
  ros::Time now = ros::Time::now();
  ros::Duration time_frame(0.002);  // HARDCODED 0.002
  double time_gap = 0.002;
  ros::Time past;
  ros::Rate rate(500.0);  // HARDCODED 500.0
  rate.sleep();

  tf::Quaternion filtered_quaternion;
  tf::Vector3 filtered_translation;

  while (publish_frames_&&ros::ok())
  {
    clearVector(rotation_vector_);
    clearVector(translation_vector_);
    for (std::string current_frame : frame_names_)
    {
      past = now;
      for (int i = 0; i < size; i++)
      {
        frame_tf_listener_.lookupTransform(PREFIX_BACKSLASH + root_frame_name_, PREFIX_BACKSLASH + current_frame,
                                           ros::Time(-time_gap), frame_transform_);
        copyValuesIntoVectors(frame_transform_, i);
      }
      filtered_quaternion = filterRotationMedian(rotation_vector_);
      filtered_translation = filterTranslationMedian(translation_vector_);
      filtered_frame_transform_.setOrigin(filtered_translation);
      filtered_frame_transform_.setRotation(filtered_quaternion);
      filtered_frame_broadcaster_.sendTransform(
          tf::StampedTransform(filtered_frame_transform_, ros::Time::now(), root_frame_name_, PREFIX_FILTERED + current_frame));
    }
  }
}

void FilterFrames::copyValuesIntoVectors(const tf::StampedTransform& transform, int i)
{
  rotation_vector_.at(0).at(i) = transform.getRotation().getX();
  rotation_vector_.at(1).at(i) = transform.getRotation().getY();
  rotation_vector_.at(2).at(i) = transform.getRotation().getZ();
  rotation_vector_.at(3).at(i) = transform.getRotation().getW();

  translation_vector_.at(0).at(i) = transform.getOrigin().getX();
  translation_vector_.at(1).at(i) = transform.getOrigin().getY();
  translation_vector_.at(2).at(i) = transform.getOrigin().getZ();
}

tf::Quaternion FilterFrames::filterRotationMedian(std::vector<std::vector<float>>& quaternion_vec)
{
  tf::Quaternion final_quat;
  int mid = quaternion_vec.at(0).size() / 2;

  for (int i = 0; i < quaternion_vec.size(); i++)
  {
    std::sort(quaternion_vec.at(i).begin(), quaternion_vec.at(i).end());
  }

  final_quat.setX(quaternion_vec.at(0).at(mid));
  final_quat.setY(quaternion_vec.at(1).at(mid));
  final_quat.setZ(quaternion_vec.at(2).at(mid));
  final_quat.setW(quaternion_vec.at(3).at(mid));

  final_quat.normalize();
  return final_quat;
}

tf::Vector3 FilterFrames::filterTranslationMedian(std::vector<std::vector<float>>& position_vec)
{
  tf::Vector3 finalTranslation;
  int mid = position_vec.at(0).size() / 2;

  for (int i = 0; i < position_vec.size(); i++)
  {
    std::sort(position_vec.at(i).begin(), position_vec.at(i).end());
  }

  finalTranslation.setX(position_vec.at(0).at(mid));
  finalTranslation.setY(position_vec.at(1).at(mid));
  finalTranslation.setZ(position_vec.at(2).at(mid));

  finalTranslation.normalize();
  return finalTranslation;
}

bool FilterFrames::getFilterPublisherStatus()
{
  return publish_frames_;
}

void FilterFrames::setFilterPublisherStatus(bool publish_filtered_frames_)
{
  publish_frames_ = publish_filtered_frames_;
}

void FilterFrames::clearVector(std::vector<std::vector<float>>& vector)
{
  for(std::vector<float> element : vector)
    element.clear();
}

void FilterFrames::resizeVector(std::vector<std::vector<float>>& vector, int elements, int size)
{
  vector.resize(elements);
  for (int i = 0; i < elements; i++)
    vector.at(i).resize(size);
}