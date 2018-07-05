/*

Copyright (c) 2004-2009, California Institute of Technology. All
rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */
// from Prosilica GigE SDK/examples/Stream/StdAfx.h
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

// from Prosilica GigE SDK/examples/Stream/Stream.cpp
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WINDOWS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

#ifdef _WINDOWS
# ifndef uint8_t
#  define uint8_t unsigned char
# endif
#endif

#if defined(_LINUX) || defined(_QNX) || defined(__APPLE__)
#include <unistd.h>
#include <time.h>
#include <signal.h>
#endif

#include "PvApi.h"

extern "C" {
#include "cam_iface.h"
#include "cam_iface_internal.h"

#ifdef _WINDOWS
#define _STDCALL __stdcall
#else
#define _STDCALL
#endif

#if 1
# define MSG(...)
#else
# define MSG(FMT,...) fprintf(stderr,"%s:%d: " FMT "\n",__func__,__LINE__,##__VA_ARGS__)
#endif

#if defined(_LINUX) || defined(_QNX) || defined(__APPLE__)
void Sleep(unsigned int time){
	struct timespec t,r;

	t.tv_sec    = time / 1000;
	t.tv_nsec   = (time % 1000) * 1000000;

	while(nanosleep(&t,&r)==-1)
		t = r;
}
#endif


#include <stdio.h>
#ifdef _WIN32
#include <Windows.h>
#include <sys/timeb.h>
#else
#include <sys/time.h>
#endif
#include <stdlib.h>
#include <time.h>

struct CCprosil; // forward declaration

// keep functable in sync across backends
typedef struct {
	cam_iface_constructor_func_t construct;
	void (*destruct)(struct CamContext*);

	void (*CCprosil)(struct CCprosil*,int,int,int,const char*);
	void (*close)(struct CCprosil*);
	void (*start_camera)(struct CCprosil*);
	void (*stop_camera)(struct CCprosil*);
	void (*get_num_camera_properties)(struct CCprosil*,int*);
	void (*get_camera_property_info)(struct CCprosil*,
		                             int,
		                             CameraPropertyInfo*);
	void (*get_camera_property)(struct CCprosil*,int,long*,int*);
	void (*set_camera_property)(struct CCprosil*,int,long,int);
	void (*grab_next_frame_blocking)(struct CCprosil*,
		                             uint8_t *,
		                             float);
	void (*grab_next_frame_blocking_with_stride)(struct CCprosil*,
		                                         uint8_t*,
		                                         intptr_t,
		                                         float);
	void (*point_next_frame_blocking)(struct CCprosil*,uint8_t**,float);
	void (*unpoint_frame)(struct CCprosil*);
	void (*get_last_timestamp)(struct CCprosil*,double*);
	void (*get_last_framenumber)(struct CCprosil*,unsigned long*);
	void (*get_num_trigger_modes)(struct CCprosil*,int*);
	void (*get_trigger_mode_string)(struct CCprosil*,int,char*,int);
	void (*get_trigger_mode_number)(struct CCprosil*,int*);
	void (*set_trigger_mode_number)(struct CCprosil*,int);
	void (*get_frame_roi)(struct CCprosil*,int*,int*,int*,int*);
	void (*set_frame_roi)(struct CCprosil*,int,int,int,int);
	void (*get_max_frame_size)(struct CCprosil*,int*,int*);
	void (*get_buffer_size)(struct CCprosil*,int*);
	void (*get_framerate)(struct CCprosil*,float*);
	void (*set_framerate)(struct CCprosil*,float);
	void (*get_num_framebuffers)(struct CCprosil*,int*);
	void (*set_num_framebuffers)(struct CCprosil*,int);
} CCprosil_functable;

typedef struct CCprosil {
	CamContext inherited;
	int num_buffers;
	int buf_size; // current buffer size (number of bytes)
	unsigned long malloced_buf_size; // maximum buffer size (number of bytes)
	int current_height;
	intptr_t current_width;
	int max_height;
	int max_width;
	tPvFrame** frames;
	int frame_number_currently_waiting_for;
	long last_framecount; // same type as Prosilica's FrameCount in struct tPvFrame
	long frame_epoch;
	double frame_epoch_start;
#ifndef CIPROSIL_TIME_HOST
	u_int64_t last_timestamp;
	double timestamp_tick;
#else
	double last_timestamp;
#endif // #ifndef CIPROSIL_TIME_HOST
	int exposure_mode_number;
} CCprosil;


// forward declarations
CCprosil* CCprosil_construct( int device_number, int NumImageBuffers,
                              int mode_number, const char *interface);
void delete_CCprosil(struct CCprosil*);

void CCprosil_CCprosil(struct CCprosil*,int,int,int,const char*);
void CCprosil_close(struct CCprosil*);
void CCprosil_start_camera(struct CCprosil*);
void CCprosil_stop_camera(struct CCprosil*);
void CCprosil_get_num_camera_properties(struct CCprosil*,int*);
void CCprosil_get_camera_property_info(struct CCprosil*,
                              int,
                              CameraPropertyInfo*);
void CCprosil_get_camera_property(struct CCprosil*,int,long*,int*);
void CCprosil_set_camera_property(struct CCprosil*,int,long,int);
void CCprosil_grab_next_frame_blocking(struct CCprosil*,
                              uint8_t*,
                              float);
void CCprosil_grab_next_frame_blocking_with_stride(struct CCprosil*,
                                          uint8_t*,
                                          intptr_t,
                                          float);
void CCprosil_point_next_frame_blocking(struct CCprosil*,uint8_t**,float);
void CCprosil_unpoint_frame(struct CCprosil*);
void CCprosil_get_last_timestamp(struct CCprosil*,double*);
void CCprosil_get_last_framenumber(struct CCprosil*,unsigned long*);
void CCprosil_get_num_trigger_modes(struct CCprosil*,int*);
void CCprosil_get_trigger_mode_string(struct CCprosil*,int,char*,int);
void CCprosil_get_trigger_mode_number(struct CCprosil*,int*);
void CCprosil_set_trigger_mode_number(struct CCprosil*,int);
void CCprosil_get_frame_roi(struct CCprosil*,int*,int*,int*,int*);
void CCprosil_set_frame_roi(struct CCprosil*,int,int,int,int);
void CCprosil_get_max_frame_size(struct CCprosil*,int*,int*);
void CCprosil_get_buffer_size(struct CCprosil*,int*);
void CCprosil_get_framerate(struct CCprosil*,float*);
void CCprosil_set_framerate(struct CCprosil*,float);
void CCprosil_get_num_framebuffers(struct CCprosil*,int*);
void CCprosil_set_num_framebuffers(struct CCprosil*,int);

CCprosil_functable CCprosil_vmt = {
	(cam_iface_constructor_func_t)CCprosil_construct,
	(void (*)(CamContext*))delete_CCprosil,
	CCprosil_CCprosil,
	CCprosil_close,
	CCprosil_start_camera,
	CCprosil_stop_camera,
	CCprosil_get_num_camera_properties,
	CCprosil_get_camera_property_info,
	CCprosil_get_camera_property,
	CCprosil_set_camera_property,
	CCprosil_grab_next_frame_blocking,
	CCprosil_grab_next_frame_blocking_with_stride,
	CCprosil_point_next_frame_blocking,
	CCprosil_unpoint_frame,
	CCprosil_get_last_timestamp,
	CCprosil_get_last_framenumber,
	CCprosil_get_num_trigger_modes,
	CCprosil_get_trigger_mode_string,
	CCprosil_get_trigger_mode_number,
	CCprosil_set_trigger_mode_number,
	CCprosil_get_frame_roi,
	CCprosil_set_frame_roi,
	CCprosil_get_max_frame_size,
	CCprosil_get_buffer_size,
	CCprosil_get_framerate,
	CCprosil_set_framerate,
	CCprosil_get_num_framebuffers,
	CCprosil_set_num_framebuffers
};


// If the following is defined, we get time from the host computer clock.
//#define CIPROSIL_TIME_HOST

double ciprosil_floattime() {
#ifdef _WIN32
#if _MSC_VER == 1310
	struct _timeb t;
	_ftime(&t);
	return (double)t.time + (double)t.millitm * (double)0.001;
#else
	struct _timeb t;
	if(_ftime_s(&t)==0) {
		return (double)t.time + (double)t.millitm * (double)0.001;
	}else{
		return 0.0;
	}
#endif
#else
	struct timeval t;
	if (gettimeofday(&t, (struct timezone *)NULL) == 0)
		return (double)t.tv_sec + t.tv_usec*0.000001;
	else
		return 0.0;
#endif
}

#ifdef MEGA_BACKEND
# define BACKEND_GLOBAL(m) prosilica_gige_##m
#else
# define BACKEND_GLOBAL(m) m
#endif


/* globals -- allocate space */
u_int64_t BACKEND_GLOBAL(prev_ts_uint64); //tmp

cam_iface_thread_local int BACKEND_GLOBAL(cam_iface_error)=0;

#define CAM_IFACE_MAX_ERROR_LEN 255
cam_iface_thread_local char BACKEND_GLOBAL(cam_iface_error_string)[CAM_IFACE_MAX_ERROR_LEN];
cam_iface_thread_local char BACKEND_GLOBAL(cam_iface_backend_string)[CAM_IFACE_MAX_ERROR_LEN];

#define PV_MAX_ENUM_LEN 32

/* global variables */
#define PV_MAX_NUM_CAMERAS 1
#define PV_MAX_NUM_BUFFERS 80
static int BACKEND_GLOBAL(num_cameras) = 0;
static tPvCameraInfo BACKEND_GLOBAL(camera_list)[PV_MAX_NUM_CAMERAS];

// circular buffer that takes advantage of 8-bit rollover
tPvFrame* BACKEND_GLOBAL(frames_ready_list_cam0)[256];
uint8_t BACKEND_GLOBAL(frames_ready_cam0_write_idx)=0;
uint8_t BACKEND_GLOBAL(frames_ready_cam0_read_idx)=0;
uint8_t BACKEND_GLOBAL(frames_ready_cam0_num)=0;

#define PV_NUM_ATTR 2
const char *BACKEND_GLOBAL(pv_attr_strings)[PV_NUM_ATTR] = {
  "gain",
  "shutter" // exposure
};
#define PV_ATTR_GAIN 0
#define PV_ATTR_SHUTTER 1


// from PvApi.h
#define PV_ERROR_NUM 22
const char *BACKEND_GLOBAL(pv_error_strings)[PV_ERROR_NUM] = {
  "No error",
  "Unexpected camera fault",
  "Unexpected fault in PvApi or driver",
  "Camera handle is invalid",
  "Bad parameter to API call",
  "Sequence of API calls is incorrect",
  "Camera or attribute not found",
  "Camera cannot be opened in the specified mode",
  "Camera was unplugged",
  "Setup is invalid (an attribute is invalid)",
  "System/network resources or memory not available",
  "1394 bandwidth not available",
  "Too many frames on queue",
  "Frame buffer is too small",
  "Frame cancelled by user",
  "The data for the frame was lost",
  "Some data in the frame is missing",
  "Timeout during wait",
  "Attribute value is out of the expected range",
  "Attribute is not this type (wrong access function)",
  "Attribute write forbidden at this time",
  "Attribute is not available at this time"
};

#ifdef MEGA_BACKEND
# define CAM_IFACE_ERROR_FORMAT(m) \
	cam_iface_snprintf(prosilica_gige_cam_iface_error_string\
		,CAM_IFACE_MAX_ERROR_LEN, "%s (%d): %s\n",__FILE__,__LINE__,(m)\
	);
#define CAM_IFACE_THROW_ERROR(m)                        \
  {                                                     \
    prosilica_gige_cam_iface_error = -1;                               \
    CAM_IFACE_ERROR_FORMAT((m));                        \
    return;                                             \
  }
#define CAM_IFACE_THROW_ERRORV(m)                       \
  {                                                     \
    prosilica_gige_cam_iface_error = -1;                               \
    CAM_IFACE_ERROR_FORMAT((m));                        \
    return NULL;                                                \
  }

#else

# define CAM_IFACE_ERROR_FORMAT(m) \
	cam_iface_snprintf(cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN,\
		"%s (%d): %s\n",__FILE__,__LINE__,(m)\
	);
#define CAM_IFACE_THROW_ERROR(m)                        \
  {                                                     \
    cam_iface_error = -1;                               \
    CAM_IFACE_ERROR_FORMAT((m));                        \
    return;                                             \
  }
#define CAM_IFACE_THROW_ERRORV(m)                       \
  {                                                     \
    cam_iface_error = -1;                               \
    CAM_IFACE_ERROR_FORMAT((m));                        \
    return NULL;                                                \
  }

#endif


#define CHECK_CC(m)                                                     \
  if (!(m)) {                                                           \
    CAM_IFACE_THROW_ERROR("no CamContext specified (NULL argument)");   \
  }

#define NOT_IMPLEMENTED CAM_IFACE_THROW_ERROR("not yet implemented");

#ifdef MEGA_BACKEND
#define CAM_IFACE_CHECK_DEVICE_NUMBER(m)                                \
  if ( ((m)<0) | ((m)>=prosilica_gige_num_cameras) ) {                                 \
    prosilica_gige_cam_iface_error = -1;                                               \
    CAM_IFACE_ERROR_FORMAT("invalid device_number");                    \
    return;                                                             \
  }
#else
#define CAM_IFACE_CHECK_DEVICE_NUMBER(m)                                \
  if ( ((m)<0) | ((m)>=num_cameras) ) {                                 \
    cam_iface_error = -1;                                               \
    CAM_IFACE_ERROR_FORMAT("invalid device_number");                    \
    return;                                                             \
  }
#endif

#ifdef MEGA_BACKEND
#define CAM_IFACE_CHECK_DEVICE_NUMBERV(m)                               \
  if ( ((m)<0) | ((m)>=prosilica_gige_num_cameras) ) {                                 \
    prosilica_gige_cam_iface_error = -1;                                               \
    CAM_IFACE_ERROR_FORMAT("invalid device_number");                    \
    return NULL;                                                        \
  }
#else
#define CAM_IFACE_CHECK_DEVICE_NUMBERV(m)                               \
  if ( ((m)<0) | ((m)>=num_cameras) ) {                                 \
    cam_iface_error = -1;                                               \
    CAM_IFACE_ERROR_FORMAT("invalid device_number");                    \
    return NULL;                                                        \
  }
#endif

#ifdef MEGA_BACKEND
#define CIPVCHK(err) {                                                  \
  tPvErr m = err;                                                       \
  if (m!=ePvErrSuccess) {                                               \
    prosilica_gige_cam_iface_error = CAM_IFACE_GENERIC_ERROR;                          \
    if (m==ePvErrTimeout) {                                             \
      prosilica_gige_cam_iface_error = CAM_IFACE_FRAME_TIMEOUT;                        \
    }                                                                   \
    if (m<PV_ERROR_NUM) {                                               \
      cam_iface_snprintf(prosilica_gige_cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: %s\n",__FILE__,__LINE__, \
                         m,                                             \
                         prosilica_gige_pv_error_strings[m]);                          \
    } else {                                                            \
      cam_iface_snprintf(prosilica_gige_cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: (unknown error)\n", \
                         __FILE__,__LINE__,                             \
                         m);                                            \
    }                                                                   \
    return;                                                             \
  }                                                                     \
  }
#else
#define CIPVCHK(err) {                                                  \
  tPvErr m = err;                                                       \
  if (m!=ePvErrSuccess) {                                               \
    cam_iface_error = CAM_IFACE_GENERIC_ERROR;                          \
    if (m==ePvErrTimeout) {                                             \
      cam_iface_error = CAM_IFACE_FRAME_TIMEOUT;                        \
    }                                                                   \
    if (m<PV_ERROR_NUM) {                                               \
      cam_iface_snprintf(cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: %s\n",__FILE__,__LINE__, \
                         m,                                             \
                         pv_error_strings[m]);                          \
    } else {                                                            \
      cam_iface_snprintf(cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: (unknown error)\n", \
                         __FILE__,__LINE__,                             \
                         m);                                            \
    }                                                                   \
    return;                                                             \
  }                                                                     \
  }
#endif

#ifdef MEGA_BACKEND
#define CIPVCHKV(err) {                                                 \
  tPvErr m = err;                                                       \
  if (m!=ePvErrSuccess) {                                               \
    prosilica_gige_cam_iface_error = CAM_IFACE_GENERIC_ERROR;                          \
    if (m==ePvErrTimeout) {                                             \
      prosilica_gige_cam_iface_error = CAM_IFACE_FRAME_TIMEOUT;                        \
    }                                                                   \
    if (m<PV_ERROR_NUM) {                                               \
      cam_iface_snprintf(prosilica_gige_cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: %s\n",__FILE__,__LINE__, \
                         m,                                             \
                         prosilica_gige_pv_error_strings[m]);                          \
    } else {                                                            \
      cam_iface_snprintf(prosilica_gige_cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: (unknown error)\n", \
                         __FILE__,__LINE__,                             \
                         m);                                            \
    }                                                                   \
    return NULL;                                                        \
  }                                                                     \
}
#else
#define CIPVCHKV(err) {                                                 \
  tPvErr m = err;                                                       \
  if (m!=ePvErrSuccess) {                                               \
    cam_iface_error = CAM_IFACE_GENERIC_ERROR;                          \
    if (m==ePvErrTimeout) {                                             \
      cam_iface_error = CAM_IFACE_FRAME_TIMEOUT;                        \
    }                                                                   \
    if (m<PV_ERROR_NUM) {                                               \
      cam_iface_snprintf(cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: %s\n",__FILE__,__LINE__, \
                         m,                                             \
                         pv_error_strings[m]);                          \
    } else {                                                            \
      cam_iface_snprintf(cam_iface_error_string,CAM_IFACE_MAX_ERROR_LEN, \
                         "%s (%d): Prosilica GigE err %d: (unknown error)\n", \
                         __FILE__,__LINE__,                             \
                         m);                                            \
    }                                                                   \
    return NULL;                                                        \
  }                                                                     \
}
#endif

#ifdef MEGA_BACKEND
#define INTERNAL_CHK() {                                                \
    if (prosilica_gige_cam_iface_error) {                                              \
      return;                                                           \
    }                                                                   \
  }
#else
#define INTERNAL_CHK() {                                                \
    if (cam_iface_error) {                                              \
      return;                                                           \
    }                                                                   \
  }
#endif

#ifdef MEGA_BACKEND
#define INTERNAL_CHKV() {                                               \
    if (prosilica_gige_cam_iface_error) {                                              \
      return void;                                                      \
    }                                                                   \
  }
#else
#define INTERNAL_CHKV() {                                               \
    if (cam_iface_error) {                                              \
      return void;                                                      \
    }                                                                   \
  }
#endif

void _internal_start_streaming( CCprosil * cam,
                                tPvHandle* handle_ptr
){
  // modeled after CFinderWindow::OnStart() in
  // in Prosilica's examples/SampleViewer/src/FinderWindow.cpp
  unsigned long lCapturing;
  tPvHandle iHandle = *handle_ptr;
  tPvUint32 iBytesPerFrame;

  CIPVCHK(PvCaptureQuery(iHandle,&lCapturing));
  if(lCapturing) {
    CAM_IFACE_THROW_ERROR("camera not in IDLE mode");
  }
  CIPVCHK(PvCaptureStart(iHandle));
  PvAttrUint32Get(iHandle,"TotalBytesPerFrame",&iBytesPerFrame);
  if(!iBytesPerFrame) {
    CAM_IFACE_THROW_ERROR("incorrect frame size");
  }
  cam->buf_size = iBytesPerFrame;
  if((cam->malloced_buf_size) < iBytesPerFrame) {
    CAM_IFACE_THROW_ERROR("buffer is larger than allocated memory");
  }
  for(int i=0; i<cam->num_buffers; i++) {
    cam->frames[i]->ImageBufferSize = cam->buf_size;
  }

  for (int i=0; i<cam->num_buffers; i++) {
    CIPVCHK(PvCaptureQueueFrame(*handle_ptr,cam->frames[i],NULL));
    BACKEND_GLOBAL(frames_ready_list_cam0)[BACKEND_GLOBAL(frames_ready_cam0_write_idx)] = cam->frames[i];
    BACKEND_GLOBAL(frames_ready_cam0_write_idx)++;
    BACKEND_GLOBAL(frames_ready_cam0_num)++;
  }

  CIPVCHK(PvCommandRun(*handle_ptr,"AcquisitionStart"));
}

void _internal_stop_streaming( CCprosil * cam,
                               tPvHandle* handle_ptr
){
  // modeled after CFinderWindow::OnStop() in
  // in Prosilica's examples/SampleViewer/src/FinderWindow.cpp
  unsigned long lCapturing;
  tPvHandle iHandle = *handle_ptr;

  CIPVCHK(PvCaptureQuery(iHandle,&lCapturing));
  if(lCapturing) {
    CIPVCHK(PvCommandRun(*handle_ptr,"AcquisitionStop"));

    // According to the PvAPI manual, this should follow the
    // PvCaptureQueueClear() call, but this order is what their sample
    // code does.
    CIPVCHK(PvCaptureEnd(iHandle));

    // Unintelligible comment from Prosilica code:
    // then dequeue all the frames still in the queue (we
    // will ignore any error as the capture was stopped anyway)
    CIPVCHK(PvCaptureQueueClear(iHandle));
  } else {
    // Comment from Prosilica code:
    // then dequeue all the frame still in the queue
    // in case there is any left in it and that the camera
    // was unplugged (we will ignore any error as the
    // capture was stopped anyway)
    CIPVCHK(PvCaptureQueueClear(iHandle));
  }
}

#include "cam_iface_prosilica_gige.h"

const char *BACKEND_METHOD(cam_iface_get_driver_name)() {
  unsigned long major, minor;
  PvVersion(&major,&minor);
  cam_iface_snprintf(BACKEND_GLOBAL(cam_iface_backend_string),CAM_IFACE_MAX_ERROR_LEN,
                     "prosilica_gige (%lu.%lu)",major,minor);
  return BACKEND_GLOBAL(cam_iface_backend_string);
}

void BACKEND_METHOD(cam_iface_clear_error)() {
  BACKEND_GLOBAL(cam_iface_error) = 0;
}

int BACKEND_METHOD(cam_iface_have_error)() {
  return BACKEND_GLOBAL(cam_iface_error);
}

const char * BACKEND_METHOD(cam_iface_get_error_string)() {
  return BACKEND_GLOBAL(cam_iface_error_string);
}

const char* BACKEND_METHOD(cam_iface_get_api_version)() {
  return CAM_IFACE_API_VERSION;
}

void BACKEND_METHOD(cam_iface_startup)() {
  unsigned long major, minor;

  PvVersion(&major,&minor);
#if defined(CAMIFACE_PROSIL_MAJOR)
  if (major!=CAMIFACE_PROSIL_MAJOR){
    CAM_IFACE_THROW_ERROR("Prosilica library version mismatch");
  }
  if (minor!=CAMIFACE_PROSIL_MINOR){
    CAM_IFACE_THROW_ERROR("Prosilica library version mismatch");
  }
  MSG("libcamiface compiled with and loaded PvAPI version %d.%d",
          major,minor);
#else
  MSG("libcamiface loaded PvAPI version %ld.%ld",
          major,minor);
#endif

  CIPVCHK(PvInitialize());

  for (int i=0;i<4;i++) {
    if (PvCameraCount()) { // wait for a camera for 4*250 msec = 1 sec
      break;
    }
    Sleep(250);
  }

  // get list of reachable cameras
  unsigned long ul_nc, numCamerasAvail;
  ul_nc = PvCameraList(BACKEND_GLOBAL(camera_list), PV_MAX_NUM_CAMERAS, &numCamerasAvail);

  if (ul_nc != numCamerasAvail) {
    CAM_IFACE_THROW_ERROR("more cameras available than PV_MAX_NUM_CAMERAS");
  }

  if (ul_nc < PV_MAX_NUM_CAMERAS) {
    MSG("trying unreachable cameras...");
    ul_nc += PvCameraListUnreachable(&BACKEND_GLOBAL(camera_list)[ul_nc],
                                     PV_MAX_NUM_CAMERAS-ul_nc,
                                     NULL);
  }

  BACKEND_GLOBAL(num_cameras) = (int)ul_nc; // cast to integer
}


void BACKEND_METHOD(cam_iface_shutdown)() {
  PvUnInitialize();
}


int BACKEND_METHOD(cam_iface_get_num_cameras)() {
  return BACKEND_GLOBAL(num_cameras);
}


void BACKEND_METHOD(cam_iface_get_camera_info)(int device_number, Camwire_id *out_camid) {
  CAM_IFACE_CHECK_DEVICE_NUMBER(device_number);
  if (out_camid==NULL) { CAM_IFACE_THROW_ERROR("return structure NULL"); }

  cam_iface_snprintf(out_camid->vendor, CAMWIRE_ID_MAX_CHARS, "Prosilica");
  cam_iface_snprintf(out_camid->model, CAMWIRE_ID_MAX_CHARS, "%s", BACKEND_GLOBAL(camera_list)[device_number].DisplayName);
  cam_iface_snprintf(out_camid->chip, CAMWIRE_ID_MAX_CHARS, "%llXh", (long long unsigned int)BACKEND_GLOBAL(camera_list)[device_number].UniqueId);
}


typedef struct{
  CameraPixelCoding coding;
  const char *pixeldepthname;
  const char *name;
  unsigned depth;
} ProsilicaCodingMapping;

#define ENTRY(enum, name, depth) { CAM_IFACE_##enum, #name, #name, depth }

/* other prosilica cameras might have more modes...? */
ProsilicaCodingMapping prosilica_gige_pixel_coding_mapping[] = {
  ENTRY(MONO8,        Mono8,        8),
  ENTRY(MONO16,       Mono16,       16),
  ENTRY(MONO12PACKED, Mono12Packed, 12),
};

#undef ENTRY

#define PROSILICA_GIGE_N_PIXEL_CODINGS                      \
((int)(   sizeof(prosilica_gige_pixel_coding_mapping)       \
        / sizeof(prosilica_gige_pixel_coding_mapping[0])    )  )


void BACKEND_METHOD(cam_iface_get_num_modes)(int device_number, int *num_modes) {
  CAM_IFACE_CHECK_DEVICE_NUMBER(device_number);
  *num_modes = PROSILICA_GIGE_N_PIXEL_CODINGS;
}


void BACKEND_METHOD(cam_iface_get_mode_string)(int device_number,
                               int mode_number,
                               char* mode_string,
                               int mode_string_maxlen) {
  CAM_IFACE_CHECK_DEVICE_NUMBER(device_number);
#if 0
  /* can't get sensor pixels without connecting to camera */
  unsigned long w,h;
  PvAttrUint32Get(Camera, "SensorWidth", &w);
  PvAttrUint32Get(Camera, "SensorHeight", &h);
  cam_iface_snprintf(mode_string, mode_string_maxlen, "%u x %u: %s",
    w, h, prosilica_gige_pixel_coding_mapping[mode_number].name
  );
#endif
  cam_iface_snprintf(mode_string, mode_string_maxlen, "%s",
    prosilica_gige_pixel_coding_mapping[mode_number].name
  );
}


cam_iface_constructor_func_t BACKEND_METHOD(cam_iface_get_constructor_func)(int device_number) {
  return (CamContext* (*)(int, int, int, const char*))CCprosil_construct;
}


CCprosil* CCprosil_construct( int device_number, int NumImageBuffers,
                                 int mode_number, const char *interface
){
  CCprosil *cam = NULL;
  cam = new CCprosil; // C++ equivalent to malloc
  memset(cam,0,sizeof(CCprosil));
  CCprosil_CCprosil(cam, device_number,NumImageBuffers, mode_number,interface);
  return cam;
}


void CCprosil_CCprosil(CCprosil * cam, int device_number, int NumImageBuffers,
                        int mode_number,const char *interface
){
	// call parent
	CamContext_CamContext((CamContext*)cam,device_number,NumImageBuffers,mode_number,interface); // XXX cast error?
	cam->inherited.vmt = (CamContext_functable*)&CCprosil_vmt;

	CAM_IFACE_CHECK_DEVICE_NUMBER(device_number);

	if (mode_number < 0 || mode_number >= PROSILICA_GIGE_N_PIXEL_CODINGS){
		BACKEND_GLOBAL(cam_iface_error) = -1;
		CAM_IFACE_THROW_ERROR("invalid mode_number");
		return;
	}

	cam->inherited.device_number = device_number;
	cam->inherited.backend_extras = NULL;

	tPvHandle* handle_ptr = new tPvHandle; // C++ equivalent to malloc
	CIPVCHK(PvCameraOpen(BACKEND_GLOBAL(camera_list)[device_number].UniqueId,
		                ePvAccessMaster,
		                handle_ptr ));
	cam->inherited.cam = (void*)handle_ptr; // save pointer

	// Check firmware version
	unsigned long FirmwareVerMajor = 0;
	unsigned long FirmwareVerMinor = 0;
	unsigned long FirmwareVerBuild = 0;

	CIPVCHK(PvAttrUint32Get(*handle_ptr,"FirmwareVerMajor",&FirmwareVerMajor));
	CIPVCHK(PvAttrUint32Get(*handle_ptr,"FirmwareVerMinor",&FirmwareVerMinor));
	CIPVCHK(PvAttrUint32Get(*handle_ptr,"FirmwareVerBuild",&FirmwareVerBuild));

	// MSG("firmware %d %d %d\n",FirmwareVerMajor,FirmwareVerMinor,FirmwareVerBuild);

	if ( ! ((FirmwareVerMajor >= 1) && (FirmwareVerMinor >= 24) ) ) {
		CAM_IFACE_THROW_ERROR("firmware too old - see http://www.prosilica.com/support/gige/ge_download.html");
	}

	const char *attr_names[] = {
		"Width"
		,"ExposureValue"
		,"FrameStartTriggerMode"
		,"RegionX"
		,"FrameRate"
		,"PacketSize"
		,"PixelFormat"
	};
	const int attr_names_size = sizeof(attr_names)/sizeof(const char *);

	tPvAttributeInfo attrInfo;
	for(int i=0;i<attr_names_size;i++) {
		MSG("%s",attr_names[i]);
		CIPVCHK(PvAttrInfo(*handle_ptr,attr_names[i],&attrInfo));
		MSG("     impact: %s",attrInfo.Impact);
		MSG("     category: %s",attrInfo.Category);
		if(attrInfo.Flags & ePvFlagRead) {
			MSG("       Read access is permitted");
		}
		if(attrInfo.Flags & ePvFlagWrite) {
			MSG("       Write access is permitted");
		}
		if(attrInfo.Flags & ePvFlagVolatile) {
			MSG("       The camera may change the value any time");
		}
		if(attrInfo.Flags & ePvFlagConst) {
			MSG("       Value is read only and never changes");
		}
	}

	// code to adjust packet size, taken from SampleViewer -- JP May 2009.
	if(!BACKEND_METHOD(cam_iface_have_error)()){
		MSG("Setting PacketSize automatically...");
		tPvUint32 lMaxSize = 8228;
		// get the last packet size set on the camera
		CIPVCHK(PvAttrUint32Get(*handle_ptr,"PacketSize",&lMaxSize));
		// adjust the packet size according to the current network capacity
		CIPVCHK(PvCaptureAdjustPacketSize(*handle_ptr,lMaxSize));
	}

	/*
	if (NumImageBuffers!=5) {
	MSG("forcing num_buffers to 5 for performance reasons"); // seems to work well - ADS 20061204
	NumImageBuffers = 5;
	}
	*/

	// modified now to handle multiple bit depths...
	cam->inherited.coding = prosilica_gige_pixel_coding_mapping[mode_number].coding;
	cam->inherited.depth = prosilica_gige_pixel_coding_mapping[mode_number].depth;
	MSG("pixel depth will be '%s'",prosilica_gige_pixel_coding_mapping[mode_number].pixeldepthname);
	CIPVCHK(PvAttrEnumSet(*handle_ptr,"PixelFormat",prosilica_gige_pixel_coding_mapping[mode_number].pixeldepthname));

	unsigned long FrameSize = 0;
	CIPVCHK(PvAttrUint32Get(*handle_ptr,"TotalBytesPerFrame",&FrameSize));
	cam->buf_size = FrameSize; // XXX should check for int overflow...

	MSG("bytes per frame = %lu",FrameSize);

#ifndef CIPROSIL_TIME_HOST
	tPvUint32 tsf;
	CIPVCHK(PvAttrUint32Get(*handle_ptr,"TimeStampFrequency",&tsf));
	cam->timestamp_tick = 1.0/((double)tsf);
	MSG("tsf %lu = dt %g",tsf,cam->timestamp_tick);
#endif // #ifndef CIPROSIL_TIME_HOST

	CCprosil_set_trigger_mode_number( cam, 0 ); // set to freerun

	/*
	char buf[PV_MAX_ENUM_LEN];
	unsigned long enum_size;
	CIPVCHK(PvAttrEnumGet(*handle_ptr,"ExposureMode",buf,PV_MAX_ENUM_LEN,&enum_size));
	fprintf(stderr,"ExposureMode enum: %s\n",buf);

	if (strncmp(buf,"FreeRun",enum_size)==0) {
	cam->exposure_mode_number=0;
	} else if (strncmp(buf,"Manual",enum_size)==0) {
	cam->exposure_mode_number=1;
	} else {
	fprintf(stderr,"ExposureMode enum: %s\n",buf);
	CAM_IFACE_THROW_ERROR("unknown ExposureMode enum");
	}
	*/

	tPvUint32 MinWidth,MaxWidth,Width;
	CIPVCHK(PvAttrRangeUint32(*handle_ptr,"Width",&MinWidth,&MaxWidth));
	CIPVCHK(PvAttrUint32Get(*handle_ptr,"Width",&Width));
	cam->current_width = Width;
	cam->max_width = MaxWidth;  // XXX should check for int overflow...
	cam->frame_epoch = 0;
	cam->frame_epoch_start = ciprosil_floattime();
	cam->last_framecount = 0;

	tPvUint32 MinHeight,MaxHeight,Height;
	CIPVCHK(PvAttrRangeUint32(*handle_ptr,"Height",&MinHeight,&MaxHeight));
	CIPVCHK(PvAttrUint32Get(*handle_ptr,"Height",&Height));
	cam->current_height = Height;  // XXX should check for int overflow...
	cam->max_height = MaxHeight;  // XXX should check for int overflow...

	cam->malloced_buf_size = (MaxWidth*MaxHeight*cam->inherited.depth + 7)/8;

	MSG("malloc = %d x %d x %d / 8 bytes = %d",MaxWidth,MaxHeight,cam->inherited.depth,cam->malloced_buf_size);

	if(NumImageBuffers>PV_MAX_NUM_BUFFERS) {
		CAM_IFACE_THROW_ERROR("requested too many buffers");
	}

	// allocate image buffers
	cam->num_buffers = NumImageBuffers;
	cam->frames = (tPvFrame**)malloc( NumImageBuffers*sizeof(tPvFrame*) );
	if(cam->frames == NULL){
		CAM_IFACE_THROW_ERROR("could not alloc frames");
	}

	for (int i=0; i<NumImageBuffers; i++) {
		cam->frames[i] = NULL;
	}
	for (int i=0; i<NumImageBuffers; i++) {
		cam->frames[i] = new tPvFrame;
		if (cam->frames[i] == NULL) {CAM_IFACE_THROW_ERROR("could not alloc frames");}
		cam->frames[i]->ImageBuffer = malloc(cam->malloced_buf_size);
		if (cam->frames[i]->ImageBuffer == NULL) {CAM_IFACE_THROW_ERROR("could not alloc buffers");}
		cam->frames[i]->ImageBufferSize = cam->buf_size;
		cam->frames[i]->AncillaryBuffer = NULL;
		cam->frames[i]->AncillaryBufferSize = 0;
		cam->frames[i]->Context[0] = (void*)i;
	}
	cam->frame_number_currently_waiting_for=0; // first frame first
}


void delete_CCprosil(CCprosil *cam) {
  CCprosil_close(cam);
  delete cam;
  cam = (CCprosil*)NULL;
}


void CCprosil_close(CCprosil *cam) {
  if (!cam) {CAM_IFACE_THROW_ERROR("no CCprosil specified (NULL argument)");}
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;

  _internal_stop_streaming(cam,handle_ptr);INTERNAL_CHK();

    if (cam->frames!=NULL) {

      //CIPVCHK(PvCaptureQueueClear(*handle_ptr));
      CIPVCHK(PvCaptureEnd(*handle_ptr));

      for (int i=0; i<(cam->num_buffers); i++) {
        if (cam->frames[i] != NULL) {
          if (cam->frames[i]->ImageBuffer != NULL) {
            free(cam->frames[i]->ImageBuffer);
            cam->frames[i]->ImageBuffer = (void*)NULL;
          }
          delete cam->frames[i];
        }
      }
      free(cam->frames);
  }

  delete handle_ptr;
  cam->inherited.cam = (void*)NULL;
}


void CCprosil_start_camera( CCprosil *cam ) {
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  _internal_start_streaming(cam,handle_ptr);INTERNAL_CHK();
}


void CCprosil_stop_camera( CCprosil *cam ) {
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  _internal_stop_streaming(cam,handle_ptr);INTERNAL_CHK();
}


void CCprosil_get_num_camera_properties(CCprosil *cam,
                                          int* num_properties) {
  CHECK_CC(cam);
  /*
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  tPvAttrListPtr attr_list;
  unsigned long num_props_pv=0;
  CIPVCHK(PvAttrList(*handle_ptr,&attr_list,&num_props_pv));
  for (int i=0;i<num_props_pv;i++) {
    MSG("attr: %d %s",i,attr_list[i]);
  }
  */
  *num_properties = PV_NUM_ATTR;
}


void CCprosil_get_camera_property_info(CCprosil *cam,
                                         int property_number,
                                         CameraPropertyInfo *info) {
	CHECK_CC(cam);
	tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;

	if (info==NULL) {
		CAM_IFACE_THROW_ERROR("no info argument specified (NULL argument)");
	}

	info->is_present = 1;

	info->min_value = 0;
	//info->max_value = min(MAX_LONG,MAX_UINT32);
	info->max_value = 0x7FFFFFFF;

	info->has_auto_mode = 1;
	info->has_manual_mode = 1;

	info->is_scaled_quantity = 0;

	info->original_value = 0;

	info->available = 1;
	info->readout_capable = 1;
	info->on_off_capable = 0;

	info->absolute_capable = 0;
	info->absolute_control_mode = 0;
	info->absolute_min_value = 0.0;
	info->absolute_max_value = 0.0;

	tPvUint32 mymin,mymax;

	switch(property_number){
	case PV_ATTR_GAIN:
		info->name = "gain";
		info->has_auto_mode = 0;
		CIPVCHK(PvAttrRangeUint32(*handle_ptr,"GainValue",&mymin,&mymax));
		info->min_value = mymin;
		info->max_value = mymax;
		break;
	case PV_ATTR_SHUTTER:
		info->name = "shutter";
		CIPVCHK(PvAttrRangeUint32(*handle_ptr,"ExposureValue",&mymin,&mymax));
		info->min_value = mymin;
		/// XXX HACK!!!
		//info->max_value = mymax;
		info->max_value = 50000;
		MSG("WARNING: artificially setting max_value of shutter to 50000");
		info->is_scaled_quantity = 1;
		info->scaled_unit_name = "usec";
		info->scale_offset = 0;
		info->scale_gain = 1e-3; // convert from microsecond to millisecond
		break;
	default:
		CAM_IFACE_THROW_ERROR("invalid property number");
		break;
	}

	return;
}


void CCprosil_get_camera_property(CCprosil *cam,
                                    int property_number,
                                    long* Value,
                                    int* Auto ) {
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;

  tPvUint32 value;
  char buf[PV_MAX_ENUM_LEN];
  unsigned long enum_size;

  switch (property_number) {
  case PV_ATTR_GAIN:
    CIPVCHK(PvAttrEnumGet(*handle_ptr,"GainMode",buf,PV_MAX_ENUM_LEN,&enum_size));
    CIPVCHK(PvAttrUint32Get(*handle_ptr,"GainValue",&value));
    *Value = value;
    if (strncmp(buf,"Manual",enum_size)==0) {
      *Auto = 0;
    } else if (strncmp(buf,"Auto",enum_size)==0) {
      *Auto = 1;
    } else {
      CAM_IFACE_THROW_ERROR("unknown enum");
    }
    break;
  case PV_ATTR_SHUTTER:
    CIPVCHK(PvAttrEnumGet(*handle_ptr,"ExposureMode",buf,PV_MAX_ENUM_LEN,&enum_size));
    CIPVCHK(PvAttrUint32Get(*handle_ptr,"ExposureValue",&value));
    *Value = value;
    if (strncmp(buf,"Manual",enum_size)==0) {
      *Auto = 0;
    } else if (strncmp(buf,"Auto",enum_size)==0) {
      *Auto = 1;
    } else {
      CAM_IFACE_THROW_ERROR("unknown enum");
    }
    break;
  default:
    CAM_IFACE_THROW_ERROR("invalid property number");
    break;
  }
  return;
}


void CCprosil_set_camera_property(CCprosil *cam,
                                    int property_number,
                                    long Value,
                                    int Auto ) {
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  const char* mode_str=NULL;
  tPvUint32 value = Value;

  switch (property_number) {
  case PV_ATTR_GAIN:
    if (Auto!=0) {
      CAM_IFACE_THROW_ERROR("auto gain not available");
    }
    CIPVCHK(PvAttrUint32Set(*handle_ptr,"GainValue",value));
    break;
  case PV_ATTR_SHUTTER:
    if (Auto==0) {
      mode_str = "Manual";
    } else {
      mode_str = "Auto";
    }
    CIPVCHK(PvAttrEnumSet(*handle_ptr,"ExposureMode",mode_str));
    CIPVCHK(PvAttrUint32Set(*handle_ptr,"ExposureValue",value));
    break;
  default:
    CAM_IFACE_THROW_ERROR("invalid property number");
    break;
  }
  return;
}


void CCprosil_grab_next_frame_blocking(CCprosil *cam
		, uint8_t *out_bytes, float timeout
){
	CHECK_CC(cam);
	CCprosil_grab_next_frame_blocking_with_stride(cam, out_bytes
		, cam->current_width * cam->inherited.depth / 8,timeout
	);
}


void CCprosil_grab_next_frame_blocking_with_stride( CCprosil *cam
		,uint8_t *out_bytes,intptr_t stride0,float timeout
){
	CHECK_CC(cam);
	tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
	tPvFrame* frame;
	unsigned long pvTimeout;
	double now;
	bool recent_rollover;

	if(timeout < 0){
		pvTimeout = PVINFINITE;
	}else{
		pvTimeout = (unsigned long)ceilf(timeout*1000.0f); // convert to msec
	}

	frame = BACKEND_GLOBAL(frames_ready_list_cam0)[BACKEND_GLOBAL(frames_ready_cam0_read_idx)];

	if (frame==NULL) CAM_IFACE_THROW_ERROR("internal cam_iface error: frame not allocated");

	CIPVCHK(PvCaptureWaitForFrameDone(*handle_ptr,frame,pvTimeout));

	BACKEND_GLOBAL(frames_ready_cam0_read_idx)++;
	BACKEND_GLOBAL(frames_ready_cam0_num)--;

	size_t wb = frame->Width;
	int height = frame->Height;

	unsigned stride = (wb * cam->inherited.depth + 7) / 8;
	if(stride0 == stride){ // same stride
		MSG("stride0 = %u, stride = %u", stride0, stride);
		for (int row=0;row<height;row++) {
			memcpy((void*)(
				out_bytes+row*stride0), //dest
				(const void*)( ((intptr_t)(frame->ImageBuffer)) + row*wb),//src
				stride //size
			);
		}
	}

	if (getenv("PROSILICA_BACKEND_DEBUG")!=NULL) {
		fprintf(stderr,"frame->FrameCount %lu\n",frame->FrameCount);
	}

	now = ciprosil_floattime();
	recent_rollover = (now - (cam->frame_epoch_start) <= 30.0);

	// FrameCount can rollover, but we don't want that
	if ( ((cam->last_framecount-1)%(long)(0xFFFF)) > (long)0xFF00) {
		if ( ((long)frame->FrameCount) < (long)0xFF00 ) {
			if (!recent_rollover) {
				// wait 30 seconds before allowing rollover again
				cam->frame_epoch++;
				cam->frame_epoch_start = now;
			}
		}
	}

	if (recent_rollover) {
		if ( ((long)frame->FrameCount) > (long)0xFF00 ) {
		  // These frames are probably coming in from before rollover.
		  cam->last_framecount = ((cam->frame_epoch-1)*(long)0xFFFF)+
			((long)frame->FrameCount);
		} else {
		  cam->last_framecount = ((cam->frame_epoch)*(long)0xFFFF)+
			((long)frame->FrameCount);
		}
	} else {
		cam->last_framecount = ((cam->frame_epoch)*(long)0xFFFF)+
		  ((long)frame->FrameCount);
	}

	if (getenv("PROSILICA_BACKEND_DEBUG")!=NULL) {
		fprintf(stderr,"cam->last_framecount %ld\n",cam->last_framecount);
	}

#ifndef CIPROSIL_TIME_HOST
	u_int64_t ts_uint64;
	ts_uint64 = (((u_int64_t)(frame->TimestampHi))<<32) + (frame->TimestampLo);
	int64_t dif64; //tmp
	dif64=ts_uint64-BACKEND_GLOBAL(prev_ts_uint64);
	BACKEND_GLOBAL(prev_ts_uint64) = ts_uint64;

	MSG("got it                         (ts %lu)    (diff %ld)!",ts_uint64,dif64);
	cam->last_timestamp = ts_uint64;
#else // #ifndef CIPROSIL_TIME_HOST
	cam->last_timestamp = ciprosil_floattime();
#endif // #ifndef CIPROSIL_TIME_HOST

	tPvErr oldstatus = frame->Status;

	//if (requeue_int==0) {
	// re-queue frame buffer
	CIPVCHK(PvCaptureQueueFrame(*handle_ptr,frame,NULL));
	//    printf("queued frame %d\n",int(frame->Context[0]));
	BACKEND_GLOBAL(frames_ready_list_cam0)[BACKEND_GLOBAL(frames_ready_cam0_write_idx)] = frame;
	BACKEND_GLOBAL(frames_ready_cam0_write_idx)++;
	BACKEND_GLOBAL(frames_ready_cam0_num)++;
	//}

	if(oldstatus == ePvErrDataMissing) {
		BACKEND_GLOBAL(cam_iface_error) = CAM_IFACE_FRAME_DATA_MISSING_ERROR;
		CAM_IFACE_ERROR_FORMAT("frame data missing");
		return;
	}

	if(oldstatus == ePvErrDataLost) {
		BACKEND_GLOBAL(cam_iface_error) = CAM_IFACE_FRAME_DATA_LOST_ERROR;
		CAM_IFACE_ERROR_FORMAT("frame data lost");
		return;
	}
}


void CCprosil_point_next_frame_blocking( CCprosil *cam, uint8_t **buf_ptr,
                                           float timeout){
  CHECK_CC(cam);
  NOT_IMPLEMENTED;
}


void CCprosil_unpoint_frame( CCprosil *cam){
  CHECK_CC(cam);
  NOT_IMPLEMENTED;
}


void CCprosil_get_last_timestamp( CCprosil *cam, double* timestamp ) {
  CHECK_CC(cam);

#ifndef CIPROSIL_TIME_HOST
  *timestamp = (double)(cam->last_timestamp) * cam->timestamp_tick;
#else // #ifndef CIPROSIL_TIME_HOST
  *timestamp = cam->last_timestamp;
#endif // #ifndef CIPROSIL_TIME_HOST

}


void CCprosil_get_last_framenumber( CCprosil *cam, unsigned long* framenumber ){
  CHECK_CC(cam);
  *framenumber = (unsigned long)(cam->last_framecount);
  if (getenv("PROSILICA_BACKEND_DEBUG")!=NULL) {
    fprintf(stderr,"*framenumber %lu\n",*framenumber);
  }
}


void CCprosil_get_num_trigger_modes( CCprosil *cam,
                                       int *num_exposure_modes ) {
  CHECK_CC(cam);
  *num_exposure_modes = 5;
}


void CCprosil_get_trigger_mode_string( CCprosil *cam,
                                         int exposure_mode_number,
                                         char* exposure_mode_string, //output parameter
                                         int exposure_mode_string_maxlen) {
  CHECK_CC(cam);
  switch (exposure_mode_number) {
  case 0:
    cam_iface_snprintf(exposure_mode_string,exposure_mode_string_maxlen,"FreeRun");
    break;
  case 1:
    cam_iface_snprintf(exposure_mode_string,exposure_mode_string_maxlen,"SyncIn1");
    break;
  case 2:
    cam_iface_snprintf(exposure_mode_string,exposure_mode_string_maxlen,"SyncIn2");
    break;
  case 3:
    cam_iface_snprintf(exposure_mode_string,exposure_mode_string_maxlen,"SyncIn3");
    break;
  case 4:
    cam_iface_snprintf(exposure_mode_string,exposure_mode_string_maxlen,"SyncIn4");
    break;
  default:
    BACKEND_GLOBAL(cam_iface_error) = -1;
    CAM_IFACE_ERROR_FORMAT("exposure_mode_number invalid");
    return;
  }
}


void CCprosil_get_trigger_mode_number( CCprosil *cam,
                                         int *exposure_mode_number ) {
  CHECK_CC(cam);
  *exposure_mode_number = (cam->exposure_mode_number);
}


void CCprosil_set_trigger_mode_number( CCprosil *cam,
                                         int exposure_mode_number ) {
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  switch (exposure_mode_number) {
  case 0:
    CIPVCHK(PvAttrEnumSet(*handle_ptr,"FrameStartTriggerMode","Freerun"));
    break;
  case 1:
    CIPVCHK(PvAttrEnumSet(*handle_ptr,"FrameStartTriggerMode","SyncIn1"));
    break;
  case 2:
    CIPVCHK(PvAttrEnumSet(*handle_ptr,"FrameStartTriggerMode","SyncIn2"));
    break;
  case 3:
    CIPVCHK(PvAttrEnumSet(*handle_ptr,"FrameStartTriggerMode","SyncIn3"));
    break;
  case 4:
    CIPVCHK(PvAttrEnumSet(*handle_ptr,"FrameStartTriggerMode","SyncIn4"));
    break;
  default:
    CAM_IFACE_THROW_ERROR("exposure_mode_number invalid");
    break;
  }
  cam->exposure_mode_number = exposure_mode_number;
}


void CCprosil_get_frame_roi( CCprosil *cam,
                             int *left, int *top, int* width, int* height ) {
  tPvUint32 l,t;

  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  CIPVCHK(PvAttrUint32Get(*handle_ptr,"RegionX",&l));
  CIPVCHK(PvAttrUint32Get(*handle_ptr,"RegionY",&t));

  *left = l;  // XXX cast error?
  *top = t;   // XXX cast error?
  *width = cam->current_width;
  *height = cam->current_height;
}


void CCprosil_set_frame_roi( CCprosil *cam,
                             int left, int top, int width, int height ) {
  // XXX not yet done
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;

  tPvUint32 l,t;
  tPvUint32 w,h;
  w=width;// XXX should check for int overflow...
  h=height;

  l=left;// XXX should check for int overflow...
  t=top;
  _internal_stop_streaming( cam, handle_ptr);INTERNAL_CHK();
  CIPVCHK(PvAttrUint32Set(*handle_ptr,"Width",w));
  cam->current_width = width;
  CIPVCHK(PvAttrUint32Set(*handle_ptr,"Height",h));
  cam->current_height = height;

  unsigned long FrameSize = 0;
  CIPVCHK(PvAttrUint32Get(*handle_ptr,"TotalBytesPerFrame",&FrameSize));
  cam->buf_size = FrameSize; // XXX should check for int overflow...

  for (int i=0; i<cam->num_buffers; i++) {
    cam->frames[i]->ImageBufferSize = cam->buf_size;
  }

  CIPVCHK(PvAttrUint32Set(*handle_ptr,"RegionX",l));
  CIPVCHK(PvAttrUint32Set(*handle_ptr,"RegionY",t));
  _internal_start_streaming( cam, handle_ptr);INTERNAL_CHK();
}


void CCprosil_get_buffer_size( CCprosil *cam,
                                 int *size) {
  CHECK_CC(cam);
  *size = cam->buf_size;
}


void CCprosil_get_framerate( CCprosil *cam,
                               float *framerate ) {
  CHECK_CC(cam);
  tPvHandle* handle_ptr = (tPvHandle*)cam->inherited.cam;
  CIPVCHK(PvAttrFloat32Get(*handle_ptr,"FrameRate",framerate));
}


void CCprosil_set_framerate( CCprosil *cam,
                               float framerate ) {
  CHECK_CC(cam);
  CAM_IFACE_THROW_ERROR("frame rate is not settable");
}


void CCprosil_get_max_frame_size( CCprosil *cam,
                                    int *width, int *height ){
  CHECK_CC(cam);
  *width = cam->max_width;
  *height = cam->max_height;
}


void CCprosil_get_num_framebuffers( CCprosil *cam,
                                      int *num_framebuffers ) {
  CHECK_CC(cam);
  *num_framebuffers = cam->num_buffers;
}


void CCprosil_set_num_framebuffers( CCprosil *cam,
                                      int num_framebuffers ) {
  CHECK_CC(cam);
  NOT_IMPLEMENTED;
}

} // closes: extern "C"

// vim: noet:ts=4:sw=4
