cmake_minimum_required(VERSION 3.8)
project(robot_audio_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioChunk.msg"
  "msg/AudioInfo.msg"
  "msg/AudioInputState.msg"
  "msg/PlaybackState.msg"
  "msg/WakeWord.msg"
  "msg/SpeechTranscript.msg"
  "msg/SpeechState.msg"
  "srv/FinishSpeech.srv"
  "action/PlayAudio.action"
  "action/Say.action"
  DEPENDENCIES std_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
