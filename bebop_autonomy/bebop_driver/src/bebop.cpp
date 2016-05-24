/**
Software License Agreement (BSD)

\file      bebop.cpp
\authors   Mani Monajjemi <mmonajje@sfu.ca>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdexcept>
#include <cmath>
#include <string>
#include <algorithm>
#include <vector>

#include <boost/bind.hpp>
#include <boost/thread/locks.hpp>
#include <boost/make_shared.hpp>

extern "C"
{
  #include "libARCommands/ARCommands.h"
  #include "libARDiscovery/ARDiscovery.h"
  #include <libARController/ARController.h>
}

#include <bebop_driver/bebop.h>
#include "bebop_driver/BebopArdrone3Config.h"
#include "bebop_driver/bebop_video_decoder.h"

// include all callback wrappers
#include "bebop_driver/autogenerated/ardrone3_state_callbacks.h"
#include "bebop_driver/autogenerated/common_state_callbacks.h"
#include "bebop_driver/autogenerated/ardrone3_setting_callbacks.h"

/*
 * Bebop coordinate systems
 *
 * Velocities:
 *
 * +x : East
 * +y : South
 * +z : Down
 *
 * Attitude:
 *
 * +x   : forward
 * +y   : right
 * +z   : down
 * +yaw : CW
 *
 * ROS coordinate system (REP 105)
 *
 * +x   : forward
 * +y   : left
 * +z   : up
 * +yaw : CCW
 *
 * Move function (setPilotingPCMD, conforms with Attiude, except for gaz)
 *
 * +roll  : right
 * +pitch : forward
 * +gaz   : up
 * +yaw   : CW
 *
 */

namespace bebop_driver
{

const char* Bebop::LOG_TAG = "BebopSDK";

void Bebop::StateChangedCallback(eARCONTROLLER_DEVICE_STATE new_state, eARCONTROLLER_ERROR error, void *bebop_void_ptr)
{
  // TODO(mani-monaj): Log error
  Bebop* bebop_ptr_ = static_cast<Bebop*>(bebop_void_ptr);

  switch (new_state)
  {
  case ARCONTROLLER_DEVICE_STATE_STOPPED:
    ARSAL_Sem_Post(&(bebop_ptr_->state_sem_));
    break;
  case ARCONTROLLER_DEVICE_STATE_RUNNING:
    ARSAL_Sem_Post(&(bebop_ptr_->state_sem_));
    break;
  }
}

void Bebop::CommandReceivedCallback(eARCONTROLLER_DICTIONARY_KEY cmd_key,
                                    ARCONTROLLER_DICTIONARY_ELEMENT_t *element_dict_ptr,
                                    void *bebop_void_ptr)
{
  static long int lwp_id = util::GetLWPId();
  static bool lwp_id_printed = false;

  if (!lwp_id_printed)
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Command Received Callback LWP id is: %ld", lwp_id);
    lwp_id_printed = true;
  }
  Bebop* bebop_ptr = static_cast<Bebop*>(bebop_void_ptr);
  if (!bebop_ptr->IsConnected()) return;

  ARCONTROLLER_DICTIONARY_ELEMENT_t *single_element_ptr = NULL;

  if (element_dict_ptr)
  {
    // We are only interested in single key dictionaries
    HASH_FIND_STR(element_dict_ptr, ARCONTROLLER_DICTIONARY_SINGLE_KEY, single_element_ptr);

    if (single_element_ptr)
    {
      callback_map_t::iterator it = bebop_ptr->callback_map_.find(cmd_key);
      if (it != bebop_ptr->callback_map_.end())
      {
        // TODO(mani-monaj): Check if we can find the time from the packets
        it->second->Update(element_dict_ptr->arguments, ros::Time::now());
      }
    }
  }
}

// This callback is called within the same context as FrameReceivedCallback()
eARCONTROLLER_ERROR Bebop::DecoderConfigCallback(ARCONTROLLER_Stream_Codec_t codec, void *bebop_void_ptr)
{
  Bebop* bebop_ptr = static_cast<Bebop*>(bebop_void_ptr);
  if (codec.type = ARCONTROLLER_STREAM_CODEC_TYPE_H264)
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "H264 configuration packet received: #SPS: %d #PPS: %d (MP4? %d)",
                codec.parameters.h264parameters.spsSize,
                codec.parameters.h264parameters.ppsSize,
                codec.parameters.h264parameters.isMP4Compliant);

    if (!bebop_ptr->video_decoder_ptr_->SetH264Params(
          codec.parameters.h264parameters.spsBuffer,
          codec.parameters.h264parameters.spsSize,
          codec.parameters.h264parameters.ppsBuffer,
          codec.parameters.h264parameters.ppsSize))
    {
      return ARCONTROLLER_ERROR;
    }
  }
  else
  {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, LOG_TAG, "Codec type is not H264");
    return ARCONTROLLER_ERROR;
  }

  return ARCONTROLLER_OK;
}

// This Callback runs in ARCONTROLLER_Stream_ReaderThreadRun context and blocks it until it returns
eARCONTROLLER_ERROR Bebop::FrameReceivedCallback(ARCONTROLLER_Frame_t *frame, void *bebop_void_ptr)
{
  static long int lwp_id = util::GetLWPId();
  static bool lwp_id_printed = false;
  if (!lwp_id_printed)
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Frame Recv & Decode LWP id: %ld", lwp_id);
    lwp_id_printed = true;
  }

  if (!frame)
  {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, LOG_TAG, "Received frame is NULL");
    return ARCONTROLLER_ERROR_NO_VIDEO;
  }

  Bebop* bebop_ptr = static_cast<Bebop*>(bebop_void_ptr);
  if (!bebop_ptr->IsConnected()) return ARCONTROLLER_ERROR;

  // TODO(mani-monaj): Param? Fetch from Drone?
  frame->width = 640;
  frame->height = 368;

  // ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "In RECV FRAME");
  {
    boost::unique_lock<boost::mutex> lock(bebop_ptr->frame_avail_mutex_);
    if (bebop_ptr->is_frame_avail_)
    {
      ARSAL_PRINT(ARSAL_PRINT_WARNING, LOG_TAG, "Previous frame might have been missed.");
    }

    if (!bebop_ptr->video_decoder_ptr_->Decode(frame))
    {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Video decode failed");
    }
    else
    {
      bebop_ptr->is_frame_avail_ = true;
      // ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "FRAME IS READY");
      bebop_ptr->frame_avail_cond_.notify_one();
    }
  }

  return ARCONTROLLER_OK;
}


Bebop::Bebop(ARSAL_Print_Callback_t custom_print_callback):
  is_connected_(false),
  is_streaming_started_(false),
  device_ptr_(NULL),
  device_controller_ptr_(NULL),
  error_(ARCONTROLLER_OK),
  device_state_(ARCONTROLLER_DEVICE_STATE_MAX),
  video_decoder_ptr_(new bebop_driver::VideoDecoder()),
  is_frame_avail_(false)
//  out_file("/tmp/ts.txt")
{
  // Redirect all calls to AR_PRINT_* to this function if provided
  if (custom_print_callback)
    ARSAL_Print_SetCallback(custom_print_callback);

  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Bebop Cnstr()");
}

Bebop::~Bebop()
{
  // This is the last resort, the program must run Cleanup() fo
  // proper disconnection and free
  if (device_ptr_) ARDISCOVERY_Device_Delete(&device_ptr_);
  if (device_controller_ptr_) ARCONTROLLER_Device_Delete(&device_controller_ptr_);
}

void Bebop::Connect(ros::NodeHandle& nh, ros::NodeHandle& priv_nh, const std::string& bebop_ip)
{
  try
  {
    if (is_connected_) throw std::runtime_error("Already inited");

    // TODO(mani-monaj): Error checking;
    ARSAL_Sem_Init(&state_sem_, 0, 0);

    eARDISCOVERY_ERROR error_discovery = ARDISCOVERY_OK;
    device_ptr_ = ARDISCOVERY_Device_New(&error_discovery);

    if (error_discovery != ARDISCOVERY_OK)
    {
      throw std::runtime_error("Discovery failed: " + std::string(ARDISCOVERY_Error_ToString(error_discovery)));
    }

    // set/save ip
    bebop_ip_ = bebop_ip;

    // TODO(mani-monaj): Make ip and port params
    error_discovery = ARDISCOVERY_Device_InitWifi(device_ptr_,
                                                  ARDISCOVERY_PRODUCT_ARDRONE, "Bebop",
                                                  bebop_ip_.c_str(), 44444);

    if (error_discovery != ARDISCOVERY_OK)
    {
      throw std::runtime_error("Discovery failed: " + std::string(ARDISCOVERY_Error_ToString(error_discovery)));
    }

    device_controller_ptr_ = ARCONTROLLER_Device_New(device_ptr_, &error_);
    ThrowOnCtrlError(error_, "Creation of device controller failed: ");

    ARDISCOVERY_Device_Delete(&device_ptr_);

      // callback_map is not being modified after this initial update
#define UPDTAE_CALLBACK_MAP
      #include "bebop_driver/autogenerated/ardrone3_state_callback_includes.h"
      #include "bebop_driver/autogenerated/common_state_callback_includes.h"
      #include "bebop_driver/autogenerated/ardrone3_setting_callback_includes.h"
#undef UPDTAE_CALLBACK_MAP

    ThrowOnCtrlError(ARCONTROLLER_Device_Start(device_controller_ptr_), "Controller device start failed");

    ThrowOnCtrlError(
          ARCONTROLLER_Device_AddStateChangedCallback(
            device_controller_ptr_, Bebop::StateChangedCallback, reinterpret_cast<void*>(this)),
          "Registering state callback failed");
    ThrowOnCtrlError(
          ARCONTROLLER_Device_AddCommandReceivedCallback(
            device_controller_ptr_, Bebop::CommandReceivedCallback, reinterpret_cast<void*>(this)),
          "Registering command callback failed");

//    ThrowOnCtrlError(
//          ARCONTROLLER_Device_SetVideoStreamMP4Compliant(
//            device_controller_ptr_, 1),
//          "Enforcing MP4 compliancy failed");

    // The forth argument is frame timeout callback
    ThrowOnCtrlError(
          ARCONTROLLER_Device_SetVideoStreamCallbacks(
            device_controller_ptr_,  Bebop::DecoderConfigCallback, Bebop::FrameReceivedCallback,
            NULL , reinterpret_cast<void*>(this)),
          "Registering video callback failed");

    // This semaphore is touched inside the StateCallback
    ARSAL_Sem_Wait(&state_sem_);

    device_state_ = ARCONTROLLER_Device_GetState(device_controller_ptr_, &error_);
    if ((error_ != ARCONTROLLER_OK) || (device_state_ != ARCONTROLLER_DEVICE_STATE_RUNNING))
    {
      throw std::runtime_error("Waiting for device failed: " + std::string(ARCONTROLLER_Error_ToString(error_)));
    }

    // Enforce termination of video streaming ... (use Start/Stop streaming to enable/disable this)
    ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendMediaStreamingVideoEnable(
                       device_controller_ptr_->aRDrone3, 0), "Stopping video stream failed.");
  }
  catch (const std::runtime_error& e)
  {
    Cleanup();
    throw e;
  }

  is_connected_ = true;
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "BebopSDK inited, lwp_id: %ld", util::GetLWPId());
}

void Bebop::Cleanup()
{
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Bebop Cleanup()");
  if (device_controller_ptr_)
  {
    device_state_ = ARCONTROLLER_Device_GetState(device_controller_ptr_, &error_);
    if ((error_ == ARCONTROLLER_OK) && (device_state_ != ARCONTROLLER_DEVICE_STATE_STOPPED))
    {
      ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Disconnecting ...");
      error_ = ARCONTROLLER_Device_Stop(device_controller_ptr_);
      if (error_ == ARCONTROLLER_OK)
      {
        ARSAL_Sem_Wait(&state_sem_);
      }
    }
    ARCONTROLLER_Device_Delete(&device_controller_ptr_);
  }
  is_connected_ = false;
  ARSAL_Sem_Destroy(&state_sem_);
}

void Bebop::StartStreaming()
{
  if (is_streaming_started_)
  {
    ARSAL_PRINT(ARSAL_PRINT_WARNING, LOG_TAG, "Video streaming is already started ...");
    return;
  }
  try
  {
    ThrowOnInternalError("Starting video stream failed");
    // Start video streaming
    ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendMediaStreamingVideoEnable(
                       device_controller_ptr_->aRDrone3, 1), "Starting video stream failed.");
    is_streaming_started_ = true;
    ARSAL_PRINT(ARSAL_PRINT_WARNING, LOG_TAG, "Video streaming started ...");
  }
  catch (const std::runtime_error& e)
  {
    ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Failed to start video streaming ...");
    is_streaming_started_ = false;
    throw e;
  }
}

void Bebop::StopStreaming()
{
  if (!is_streaming_started_) return;
  try
  {
    ThrowOnInternalError("Stopping video stream failed");
    // Stop video streaming
    ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendMediaStreamingVideoEnable(
                       device_controller_ptr_->aRDrone3, 0), "Stopping video stream failed.");
    is_streaming_started_ = false;
  }
  catch (const std::runtime_error& e)
  {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, LOG_TAG, "Failed to stop video streaming ...");
  }
}

void Bebop::Disconnect()
{
  if (!is_connected_) return;
  if (is_streaming_started_) StopStreaming();
  Cleanup();
  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "Disconnected from Bebop ...");
}

void Bebop::RequestAllSettings()
{
  ThrowOnInternalError("Request Settings Failed");
  ThrowOnCtrlError(
        device_controller_ptr_->common->sendSettingsAllSettings(device_controller_ptr_->common),
        "Request Settings Failed");
}

void Bebop::ResetAllSettings()
{
  ThrowOnInternalError("Reset Settings Failed");
  ThrowOnCtrlError(
        device_controller_ptr_->common->sendSettingsReset(device_controller_ptr_->common),
        "Reset Settings Failed");

  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "All settings of the drone have been reset to default values.");
}

void Bebop::UpdateSettings(const BebopArdrone3Config &config)
{
  ThrowOnInternalError("Update Settings Failed");

  // For all callback_map members
  // 1) Check if they are derived from AbstractSetting type
  // 1.1) Pass the config objects to them
//  boost::lock_guard<boost::mutex> lock(callback_map_mutex_);
  for (callback_map_t::iterator it = callback_map_.begin(); it != callback_map_.end(); it++)
  {
    // Convert the base class pointer (AbstractCommand) to the derived class pointer (AbstractSetting)
    // In case of State classes, do nothing
    boost::shared_ptr<cb::AbstractSetting> setting_ptr = boost::dynamic_pointer_cast<cb::AbstractSetting>(it->second);
    if (setting_ptr)
    {
      setting_ptr->UpdateBebopFromROS(config, device_controller_ptr_);
    }
  }
}

void Bebop::Takeoff()
{
  ThrowOnInternalError("Takeoff failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingTakeOff(device_controller_ptr_->aRDrone3),
        "Takeoff failed");
}

void Bebop::Land()
{
  ThrowOnInternalError("Land failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingLanding(device_controller_ptr_->aRDrone3),
        "Land failed");
}

void Bebop::Emergency()
{
  ThrowOnInternalError("Emergency failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingEmergency(device_controller_ptr_->aRDrone3),
        "Emergency failed");
}

void Bebop::FlatTrim()
{
  ThrowOnInternalError("FlatTrim failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingFlatTrim(device_controller_ptr_->aRDrone3),
        "FlatTrim failed");
}

void Bebop::NavigateHome(const bool &start_stop)
{
  ThrowOnInternalError("Navigate home failed");
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendPilotingNavigateHome(
          device_controller_ptr_->aRDrone3, start_stop ? 1 : 0),
        "Navigate home failed");
}

void Bebop::AnimationFlip(const uint8_t &anim_id)
{
  ThrowOnInternalError("Animation failed");
  if (anim_id >= ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_MAX)
  {
    throw std::runtime_error("Inavlid animation id");
  }
  ThrowOnCtrlError(
        device_controller_ptr_->aRDrone3->sendAnimationsFlip(
          device_controller_ptr_->aRDrone3, static_cast<eARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION>(
            anim_id % ARCOMMANDS_ARDRONE3_ANIMATIONS_FLIP_DIRECTION_MAX)),
        "Navigate home failed");
}

void Bebop::Move(const double &roll, const double &pitch, const double &gaz_speed, const double &yaw_speed)
{
  // TODO(mani-monaj): Bound check
  ThrowOnInternalError("Move failure");

  // If roll or pitch value are non-zero, enabel roll/pitch flag
  const bool do_rp = !((fabs(roll) < 0.001) && (fabs(pitch) < 0.001));

  // If all values are zero, hover
  const bool do_hover = !do_rp && (fabs(yaw_speed) < 0.001) && (fabs(gaz_speed) < 0.001);

  if (do_hover)
  {
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, LOG_TAG, "STOP");
    ThrowOnCtrlError(
          device_controller_ptr_->aRDrone3->setPilotingPCMD(
            device_controller_ptr_->aRDrone3,
            0, 0, 0, 0, 0, 0));
  }
  else
  {
    ThrowOnCtrlError(
          device_controller_ptr_->aRDrone3->setPilotingPCMD(
            device_controller_ptr_->aRDrone3,
            do_rp,
            static_cast<int8_t>(roll * 100.0),
            static_cast<int8_t>(pitch * 100.0),
            static_cast<int8_t>(yaw_speed * 100.0),
            static_cast<int8_t>(gaz_speed * 100.0),
            0));
  }
}

// in degrees
void Bebop::MoveCamera(const double &tilt, const double &pan)
{
  ThrowOnInternalError("Camera Move Failure");
  ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendCameraOrientation(
                     device_controller_ptr_->aRDrone3,
                     static_cast<int8_t>(tilt),
                     static_cast<int8_t>(pan)));
}

uint32_t Bebop::GetFrontCameraFrameWidth() const
{
  return video_decoder_ptr_->GetFrameWidth();
}

uint32_t Bebop::GetFrontCameraFrameHeight() const
{
  return video_decoder_ptr_->GetFrameHeight();
}

bool Bebop::GetFrontCameraFrame(std::vector<uint8_t> &buffer, uint32_t& width, uint32_t& height) const
{
  boost::unique_lock<boost::mutex> lock(frame_avail_mutex_);

  ARSAL_PRINT(ARSAL_PRINT_DEBUG, LOG_TAG, "Waiting for frame to become available ...");
  while (!is_frame_avail_)
  {
    frame_avail_cond_.wait(lock);
  }

//  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "COPY STARTED");
  const uint32_t num_bytes = video_decoder_ptr_->GetFrameWidth() * video_decoder_ptr_->GetFrameHeight() * 3;

  buffer.resize(num_bytes);
  // New frame is ready
  std::copy(video_decoder_ptr_->GetFrameRGBRawCstPtr(),
            video_decoder_ptr_->GetFrameRGBRawCstPtr() + num_bytes,
            buffer.begin());

  width = video_decoder_ptr_->GetFrameWidth();
  height = video_decoder_ptr_->GetFrameHeight();
  is_frame_avail_ = false;
//  ARSAL_PRINT(ARSAL_PRINT_INFO, LOG_TAG, "COPY ENDED");
  return true;
}

void Bebop::TakeSnapshot()
{
  ThrowOnInternalError("Snapshot Failure");
  ThrowOnCtrlError(
    device_controller_ptr_->aRDrone3->sendMediaRecordPictureV2(
          device_controller_ptr_->aRDrone3));
}

void Bebop::ToggleVideoRecording(const bool start)
{
  ThrowOnInternalError("Video Toggle Failure");
  ThrowOnCtrlError(device_controller_ptr_->aRDrone3->sendMediaRecordVideoV2(
                     device_controller_ptr_->aRDrone3,
                     start ? ARCOMMANDS_ARDRONE3_MEDIARECORD_VIDEOV2_RECORD_START :
                             ARCOMMANDS_ARDRONE3_MEDIARECORD_VIDEOV2_RECORD_STOP));
}

void Bebop::ThrowOnInternalError(const std::string &message)
{
  if (!is_connected_ || !device_controller_ptr_)
  {
    throw std::runtime_error(message);
  }
}

void Bebop::ThrowOnCtrlError(const eARCONTROLLER_ERROR &error, const std::string &message)
{
  if (error != ARCONTROLLER_OK)
  {
    throw std::runtime_error(message + std::string(ARCONTROLLER_Error_ToString(error)));
  }
}

}  // namespace bebop_driver