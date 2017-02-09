/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>		/* getopt_long() */

#include <fcntl.h>		/* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <asm/types.h>		/* for videodev2.h */

#include <linux/videodev2.h>

#include "atomisp_obj.h"
#include "atomisp.h"
#include <pthread.h>

struct atomisp_loop {
	class CAtomispObj *main;
	class CAtomispObj *vf;
	unsigned int loop;
};

struct time_profile {
	struct timeval tv_start;
	struct timeval tv_end;
	unsigned int loop;
	const char *proc_name;
};

static char *dev_name = NULL;
static io_method io = IO_METHOD_MMAP;

/* ISP Specfic parameter */
int horizontal_flip=0;
int vertical_flip=0;
static int cam_input = 0; /* Camera ID selection */
static int main_width = 640;
static int main_height = 480;
static int main_format = V4L2_PIX_FMT_YUV420;
static int vf_width = 640;
static int vf_height = 480;
static int vf_format = V4L2_PIX_FMT_YUV420;
static int sdv_cap_width = 640;
static int sdv_cap_height = 480;
static int sdv_cap_format = V4L2_PIX_FMT_YUV420;
static int sdv_cap_vf_width = 640;
static int sdv_cap_vf_height = 480;
static int sdv_cap_vf_format = V4L2_PIX_FMT_YUV420;
static int zsl_cap_width = 640;
static int zsl_cap_height = 480;
static int zsl_cap_format = V4L2_PIX_FMT_YUV420;
static int zsl_cap_vf_width = 640;
static int zsl_cap_vf_height = 480;
static int zsl_cap_vf_format = V4L2_PIX_FMT_YUV420;
static int num_frame = 1;
static unsigned int sdv_cap_num_frame = 1;
static unsigned int zsl_cap_num_frame = 0;
static int dual_enable = 0;
static int saveFile = 1;
static int sub_stream = 0;
static int subdev_index = 0;
static int h_blank = -1;
static int v_blank = -1;
static int test_pattern = 0;
static int fps = 30;
#define MAX_STREAM_NUM	2
#define ISP_MODE_PREVIEW	0x8000
#define ISP_MODE_VIDEO		0x4000
#define ISP_MODE_STILL		0x2000
#define DBG_HV_BLANK		0x1000
static int run_mode = ISP_MODE_PREVIEW;
static bool sdv = false;
static bool zsl = false;
static char *main_out_node[] = {"/dev/video0", "/dev/video5"};
static char *post_out_node[] = {"/dev/video1", "/dev/video6"};
static char *vf_out_node[] = {"/dev/video2", "/dev/video7"};
static char *video_out_node[] = {"/dev/video3", "/dev/video8"};

char *format_to_str(int format);

static void format_convert(char *s, int *dst)
{
#define capture_format *dst
		if (!strncmp(s, "RAW10", 5))
			capture_format = V4L2_PIX_FMT_SBGGR10;
		if (!strncmp(s, "BGGR10", 6))
			capture_format = V4L2_PIX_FMT_SBGGR10;
		if (!strncmp(s, "RGGB10", 6))
			capture_format = V4L2_PIX_FMT_SRGGB10;
		if (!strncmp(s, "GRBG10", 6))
			capture_format = V4L2_PIX_FMT_SGRBG10;
		if (!strncmp(s, "GBRG10", 6))
			capture_format = V4L2_PIX_FMT_SGBRG10;
		if (!strncmp(s, "RAW8", 4))
			capture_format = V4L2_PIX_FMT_SRGGB8;
		if (!strncmp(s, "BGGR8", 5))
			capture_format = V4L2_PIX_FMT_SBGGR8;
		if (!strncmp(s, "RGGB8", 5))
			capture_format = V4L2_PIX_FMT_SRGGB8;
		if (!strncmp(s, "GRBG8", 5))
			capture_format = V4L2_PIX_FMT_SGRBG8;
		if (!strncmp(s, "GBRG8", 5))
			capture_format = V4L2_PIX_FMT_SGBRG8;
		if (!strncmp(s, "YUV420", 6))
			capture_format = V4L2_PIX_FMT_YUV420;
		if (!strncmp(s, "YVU420", 6))
			capture_format = V4L2_PIX_FMT_YVU420;
		if (!strncmp(s, "YUV422", 6))
			capture_format = V4L2_PIX_FMT_YUV422P;
		if (!strncmp(s, "YUV444", 6))
			capture_format = V4L2_PIX_FMT_YUV444;
		if (!strncmp(s, "NV16", 4))
			capture_format = V4L2_PIX_FMT_NV16;
		if (!strncmp(s, "NV61", 4))
			capture_format = V4L2_PIX_FMT_NV61;
		if (!strncmp(s, "YUYV", 4))
			capture_format = V4L2_PIX_FMT_YUYV;
		if (!strncmp(s, "UYVY", 4))
			capture_format = V4L2_PIX_FMT_UYVY;
		if (!strncmp(s, "NV12", 4))
			capture_format = V4L2_PIX_FMT_NV12;
		if (!strncmp(s, "NV21", 4))
			capture_format = V4L2_PIX_FMT_NV21;
		if (!strncmp(s, "RGB565", 6))
			capture_format = V4L2_PIX_FMT_RGB565;
		if (!strncmp(s, "RGB24", 5))
			capture_format = V4L2_PIX_FMT_RGB24;
		if (!strncmp(s, "RGB32", 5))
			capture_format = V4L2_PIX_FMT_RGB32;
}

static int capture_check_env(void)
{
	char *s = NULL;

	s = getenv("HFLIP");
	if (s != NULL)
		horizontal_flip = atoi(s);

	s = getenv("VFLIP");
	if (s != NULL)
		vertical_flip = atoi(s);

	s = getenv("INPUT");
	if (s != NULL)
		cam_input = atoi(s);

	s = getenv("M_W");
	if (s != NULL)
		main_width = atoi(s);

	s = getenv("M_H");
	if (s != NULL)
		main_height = atoi(s);

	s = getenv("M_F");
	if (s !=NULL)
		format_convert(s, &main_format);

	s = getenv("V_W");
	if (s != NULL)
		vf_width = atoi(s);

	s = getenv("V_H");
	if (s != NULL)
		vf_height = atoi(s);

	s = getenv("V_F");
	if (s !=NULL)
		format_convert(s, &vf_format);

	s = getenv("SDV_W");
	if (s != NULL)
		sdv_cap_width = atoi(s);

	s = getenv("SDV_H");
	if (s != NULL)
		sdv_cap_height = atoi(s);

	s = getenv("SDV_F");
	if (s !=NULL)
		format_convert(s, &sdv_cap_format);

	s = getenv("SDV_V_W");
	if (s != NULL)
		sdv_cap_vf_width = atoi(s);

	s = getenv("SDV_V_H");
	if (s != NULL)
		sdv_cap_vf_height = atoi(s);

	s = getenv("SDV_V_F");
	if (s !=NULL)
		format_convert(s, &sdv_cap_vf_format);

	s = getenv("ZSL_W");
	if (s != NULL)
		zsl_cap_width = atoi(s);

	s = getenv("ZSL_H");
	if (s != NULL)
		zsl_cap_height = atoi(s);

	s = getenv("ZSL_F");
	if (s !=NULL)
		format_convert(s, &zsl_cap_format);

	s = getenv("ZSL_V_W");
	if (s != NULL)
		zsl_cap_vf_width = atoi(s);

	s = getenv("ZSL_V_H");
	if (s != NULL)
		zsl_cap_vf_height = atoi(s);

	s = getenv("ZSL_V_F");
	if (s !=NULL)
		format_convert(s, &zsl_cap_vf_format);

	s = getenv("C");
	if (s != NULL)
		num_frame = atoi(s);

	s = getenv("SDV_C");
	if (s != NULL)
		sdv_cap_num_frame = atoi(s);

	s = getenv("ZSL_C");
	if (s != NULL)
		zsl_cap_num_frame = atoi(s);

	s = getenv("DUAL");
	if (s != NULL)
		dual_enable = atoi(s);

	s = getenv("MODE");
	if (s != NULL) {
		if (!strncmp(s, "STILL", 5))
			run_mode = ISP_MODE_STILL;
		if (!strncmp(s, "PREVIEW", 7))
			run_mode = ISP_MODE_PREVIEW;
		if (!strncmp(s, "VIDEO", 5))
			run_mode = ISP_MODE_VIDEO;
		if (!strncmp(s, "HVBLANK", 5))
			run_mode = DBG_HV_BLANK;
		if (!strncmp(s, "SDV", 3)) {
			run_mode = ISP_MODE_VIDEO;
			sdv = true;
		}
		if (!strncmp(s, "ZSL", 3)) {
			run_mode = ISP_MODE_PREVIEW;
			zsl = true;
		}
	}

	s = getenv("SAVE_FILE");
	if (s != NULL)
		saveFile = atoi(s);

	s = getenv("STREAM");
	if (s != NULL)
		sub_stream = atoi(s) % MAX_STREAM_NUM;

	s = getenv("SUBDEV");
	if (s != NULL)
		subdev_index = atoi(s);

	s = getenv("VBLANK");
	if (s != NULL)
		v_blank = atoi(s);

	s = getenv("HBLANK");
	if (s != NULL)
		h_blank = atoi(s);

	s = getenv("TEST_PATTERN");
	if (s != NULL)
		test_pattern = atoi(s);

	s = getenv("FPS");
	if (s != NULL)
		fps = atoi(s);

	return 0;
}

static void usage(FILE * fp, int argc, char **argv)
{
	(void)argc;

	fprintf(fp,
	       "Usage: %s [options]\n\n"
	       "Options:\n"
	       "-d | --device name   Video device name [/dev/video]\n"
	       "-h | --help          Print this message\n"
	       "-m | --mmap          Use memory mapped buffers\n"
	       "-r | --read          Use read() calls\n"
	       "-u | --userp         Use application allocated buffers\n"
	       "HFLIP=1		Enable Horizontal Flip\n"
	       "VFLIP=1		Enable Vertical Flip\n"
	       "M_W=		Main width\n"
	       "M_H=		Main height\n"
	       "M_F=		Main format\n"
	       "		RAW10|RAW8|YUV420|YVU420|YUV422|YUV444\n"
	       "		NV12|NV21|NV16|NV61|RBG565|RGB32\n"
	       "V_W=		Viewfinder width\n"
	       "V_H=		Viewfinder height\n"
	       "V_F=		Viewfinder format\n"
	       "		RAW10|RAW8|YUV420|YVU420|YUV422|YUV444\n"
	       "		NV12|NV21|NV16|NV61|RBG565|RGB32\n"
	       "SDV_W=		SDV main width\n"
	       "SDV_H=		SDV main height\n"
	       "SDV_F=		SDV main format\n"
	       "		RAW10|RAW8|YUV420|YVU420|YUV422|YUV444\n"
	       "		NV12|NV21|NV16|NV61|RBG565|RGB32\n"
	       "SDV_V_W=	SDV capture viewfinder width\n"
	       "SDV_V_H=	SDV capture viewfinder height\n"
	       "SDV_V_F=	SDV capture viewfinder format\n"
	       "		RAW10|RAW8|YUV420|YVU420|YUV422|YUV444\n"
	       "		NV12|NV21|NV16|NV61|RBG565|RGB32\n"
	       "ZSL_W=		ZSL main width\n"
	       "ZSL_H=		ZSL main height\n"
	       "ZSL_F=		ZSL main format\n"
	       "		RAW10|RAW8|YUV420|YVU420|YUV422|YUV444\n"
	       "		NV12|NV21|NV16|NV61|RBG565|RGB32\n"
	       "ZSL_V_W=	ZSL capture viewfinder width\n"
	       "ZSL_V_H=	ZSL capture viewfinder height\n"
	       "ZSL_V_F=	ZSL capture viewfinder format\n"
	       "		RAW10|RAW8|YUV420|YVU420|YUV422|YUV444\n"
	       "		NV12|NV21|NV16|NV61|RBG565|RGB32\n"
	       "MODE=		ISP running mode\n"
	       "		PREIVEW|STILL|VIDEO|HVBLANK|SDV|ZSL\n"
	       "C=		The number of frame to capture\n"
	       "SDV_C=		The number of frame to SDV capture\n"
	       "ZSL_C=		The number of frame to ZSL capture\n"
	       "DUAL=		Enable dual output for STILL and VIDEO modes\n"
	       "SAVE_FILE=	Enable image storing to file system\n"
	       "STREAM=		Select stream ID for dual stream case\n"
	       "SUBDEV=		subdev index\n"
	       "HBLANK=		set h blank in num of pixels\n"
	       "VBLANK=		set v blank in num of lines\n"
	       "FPS=		set sensor fps\n"
	       "\n"
	       "Examples:\n"
	       "M_W=1920 M_H=1080 M_F=NV12 C=1 MODE=STILL DUAL=1 INPUT=0 capture_v4l2\n"
	       "M_W=1280 M_H=720 M_F=UYVY C=1 MODE=PREVIEW INPUT=1 capture_v4l2\n"
	       "M_W=960 M_H=720 M_F=NV12 C=10 ZSL_W=4160 ZSL_H=3104 ZSL_F=NV12 ZSL_V_W=640 ZSL_V_H=480 ZSL_V_F=NV12 ZSL_C=2 MODE=ZSL INPUT=0 capture_v4l2\n"
	       "", argv[0]);
}

static const char short_options[] = "d:hmru";

static const struct option long_options[] = {
	{"device", required_argument, NULL, 'd'},
	{"help", no_argument, NULL, 'h'},
	{"mmap", no_argument, NULL, 'm'},
	{"read", no_argument, NULL, 'r'},
	{"userp", no_argument, NULL, 'u'},
	{0, 0, 0, 0}
};

static void process_basic_preview();
static void process_basic_still();
static void process_basic_video();
static void process_sdv_video();
static void process_zsl_preview();
static int dbg_h_v_blank(void);

int flip_control(class CAtomispObj & objControl, int hflip, int vflip){
	struct v4l2_control control;
    int ret;
	control.id = V4L2_CID_HFLIP;
	control.value = hflip;
	ret=objControl.xioctl(VIDIOC_S_CTRL, &control);
	control.id = V4L2_CID_VFLIP;
	control.value = vflip;
	ret|=objControl.xioctl(VIDIOC_S_CTRL, &control);
    return ret;
}

int main(int argc, char **argv)
{
	for (;;) {
		int index;
		int c;

		c = getopt_long(argc, argv, short_options, long_options,
				&index);

		if (-1 == c)
			break;

		switch (c) {
		case 0:	/* getopt_long() flag */
			break;

		case 'd':
			dev_name = optarg;
			break;

		case 'h':
			usage(stdout, argc, argv);
			exit(EXIT_SUCCESS);

		case 'm':
			io = IO_METHOD_MMAP;
			break;

		case 'r':
			io = IO_METHOD_READ;
			break;

		case 'u':
			io = IO_METHOD_USERPTR;
			break;

		default:
			usage(stderr, argc, argv);
			exit(EXIT_FAILURE);
		}
	}

	/* Get Parameter for atomisp specfic parameter */
	capture_check_env();

	switch (run_mode) {
	case ISP_MODE_PREVIEW:
		if (zsl)
			process_zsl_preview();
		else
			process_basic_preview();
		break;
	case ISP_MODE_STILL:
		process_basic_still();
		break;
	case ISP_MODE_VIDEO:
		if (sdv)
			process_sdv_video();
		else
			process_basic_video();
		break;
	case DBG_HV_BLANK:
		dbg_h_v_blank();
		break;
	}

	exit(EXIT_SUCCESS);

	return 0;
}

static void profile_start(struct time_profile *profile)
{
	gettimeofday(&profile->tv_start, NULL);
}

static void profile_end(struct time_profile *profile)
{
	double time;
	gettimeofday(&profile->tv_end, NULL);
	time = profile->tv_end.tv_sec * 1000000.0 + profile->tv_end.tv_usec;
	time -= profile->tv_start.tv_sec * 1000000.0 + profile->tv_start.tv_usec;
	printf("%s: Time=%fms FPS=%f\n", profile->proc_name, time / 1000.0,
		1000000.0 * profile->loop / time);
}

static void process_basic_preview()
{
	class CAtomispObj iAtomispObj(vf_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	int loop = num_frame;
	struct time_profile profile;
	struct v4l2_control control;
	profile.loop = loop;
	profile.proc_name = "Preview";

	printf("Preview mode: %dx%d, %s.\n", main_width, main_height,
	       format_to_str(main_format));
	/* open main output */
	iAtomispObj.open_device();

	/* Init device */
	iAtomispObj.set_format(main_width, main_height, main_format);
	iAtomispObj.init_device();
	iAtomispObj.init_buffers();

	/* Configure test pattern. */
	control.id = V4L2_CID_TEST_PATTERN;
	control.value = test_pattern;
	iAtomispObj.xioctl(VIDIOC_S_CTRL, &control);

	flip_control(iAtomispObj, horizontal_flip, vertical_flip);
	/* start capture */
	iAtomispObj.start_capturing();

	profile_start(&profile);
	iAtomispObj.mainloop(loop);
	profile_end(&profile);

	/* stop capture */
	iAtomispObj.stop_capturing();

	/* Uninit device */
	iAtomispObj.uninit_device();

	/* close device */
	iAtomispObj.close_device();

}
static void process_basic_still()
{
	class CAtomispObj iAtomispObj_main(main_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_vf(post_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	int loop = num_frame;
	struct time_profile profile;
	struct v4l2_control control;
	profile.loop = loop;
	profile.proc_name = "Still";

	printf("Still mode: %dx%d, %s.\n", main_width, main_height,
	       format_to_str(main_format));

	/* open main output */
	iAtomispObj_main.open_device();
	if (dual_enable)
		iAtomispObj_vf.open_device();

	/* Init device */
	iAtomispObj_main.set_format(main_width, main_height, main_format);
	iAtomispObj_main.init_device();
	iAtomispObj_main.init_buffers();
	if (dual_enable) {
		iAtomispObj_vf.set_format(vf_width, vf_height, vf_format);
		iAtomispObj_vf.init_device();
		iAtomispObj_vf.init_buffers();
	}

	/* Configure test pattern. */
	control.id = V4L2_CID_TEST_PATTERN;
	control.value = test_pattern;
	iAtomispObj_main.xioctl(VIDIOC_S_CTRL, &control);

	flip_control(iAtomispObj_main, horizontal_flip, vertical_flip);

	/* start capture */
	profile_start(&profile);
	if (!dual_enable) {
		iAtomispObj_main.start_capturing();
		iAtomispObj_main.mainloop(loop);
	} else {
		iAtomispObj_main.start_capturing();
		iAtomispObj_vf.start_capturing();
		while (loop--) {
			iAtomispObj_main.mainloop(1);
			iAtomispObj_vf.mainloop(1);
		}
	}
	profile_end(&profile);

	/* stop capture */
	iAtomispObj_main.stop_capturing();
	if (dual_enable)
		iAtomispObj_vf.stop_capturing();

	/* Uninit device */
	iAtomispObj_main.uninit_device();
	if (dual_enable)
		iAtomispObj_vf.uninit_device();

	/* close device */
	iAtomispObj_main.close_device();
	if (dual_enable)
		iAtomispObj_vf.close_device();
}
static void process_basic_video()
{
	class CAtomispObj iAtomispObj_main(video_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_vf(vf_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	int loop = num_frame;
	struct time_profile profile;
	struct v4l2_control control;
	struct v4l2_streamparm parm;

	profile.loop = loop;
	profile.proc_name = "Video";

	printf("Video mode: %dx%d, %s.\n", main_width, main_height,
	       format_to_str(main_format));

	/* open main output */
	iAtomispObj_main.open_device();
	if (dual_enable)
		/* Don't set input since it will power cycle the sensor and
		 * wrong resolution table is selected. */
		iAtomispObj_vf.open_device(false);

	printf("Setting fps to %d\n", fps);
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parm.parm.capture.capturemode = CI_MODE_NONE;
	parm.parm.capture.timeperframe.numerator = 1;
	parm.parm.capture.timeperframe.denominator = fps;
	iAtomispObj_main.xioctl(VIDIOC_S_PARM, &parm);

	/* Init device */
	iAtomispObj_main.set_format(main_width, main_height, main_format);
	iAtomispObj_main.init_device();
	iAtomispObj_main.init_buffers();
	if (dual_enable) {
		iAtomispObj_vf.set_format(vf_width, vf_height, vf_format);
		iAtomispObj_vf.init_device();
		iAtomispObj_vf.init_buffers();
	}

	/* Configure test pattern. */
	control.id = V4L2_CID_TEST_PATTERN;
	control.value = test_pattern;
	iAtomispObj_main.xioctl(VIDIOC_S_CTRL, &control);

	flip_control(iAtomispObj_main, horizontal_flip, vertical_flip);

	/* start capture */
	profile_start(&profile);
	if (!dual_enable) {
		iAtomispObj_main.start_capturing();
		iAtomispObj_main.mainloop(loop);
	} else {
		iAtomispObj_main.start_capturing();
		iAtomispObj_vf.start_capturing();
		while (loop--) {
			iAtomispObj_main.mainloop(1);
			iAtomispObj_vf.mainloop(1);
		}
	}
	profile_end(&profile);

	/* stop capture */
	iAtomispObj_main.stop_capturing();
	if (dual_enable)
		iAtomispObj_vf.stop_capturing();

	/* Uninit device */
	iAtomispObj_main.uninit_device();
	if (dual_enable)
		iAtomispObj_vf.uninit_device();

	/* close device */
	iAtomispObj_main.close_device();
	if (dual_enable)
		iAtomispObj_vf.close_device();
}

void* frame_loop(void *ptr)
{
	struct atomisp_loop *atomisp = (struct atomisp_loop *)ptr;
	while (atomisp->loop--) {
		atomisp->main->mainloop(1);
		if (atomisp->vf)
			atomisp->vf->mainloop(1);
	}
	return NULL;
}

static void process_sdv_video()
{
	struct v4l2_control control;
	struct v4l2_ext_controls controls;
	struct v4l2_ext_control ext_control;
	struct v4l2_streamparm parm;
	class CAtomispObj iAtomispObj_main(video_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_vf(vf_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_cap_main(main_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_cap_vf(post_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	pthread_t tid1, tid2;
	struct atomisp_loop video_loop, capture_loop;
	struct atomisp_cont_capture_conf capture_conf;
	struct time_profile video_profile, capture_profile;
	video_profile.proc_name = "Video+Preview";
	video_profile.loop = num_frame;
	capture_profile.proc_name = "Capture+VF";
	capture_profile.loop = sdv_cap_num_frame;

	printf("SDV Video mode: %dx%d, %s.\n", main_width, main_height,
	       format_to_str(main_format));
	printf("SDV Capture mode: %dx%d, %s.\n", sdv_cap_width, sdv_cap_height,
	       format_to_str(sdv_cap_format));

	/* open main output */
	iAtomispObj_main.open_device();
	iAtomispObj_vf.open_device();
	iAtomispObj_cap_main.open_device();
	iAtomispObj_cap_vf.open_device();

	/* set ISP mode */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parm.parm.capture.capturemode = CI_MODE_PREVIEW;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_PARM, &parm);
	parm.parm.capture.capturemode = run_mode;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_PARM, &parm);

	/* Init device */
	memset(&capture_conf, 0, sizeof(capture_conf));
	capture_conf.num_captures = (sdv_cap_num_frame ? sdv_cap_num_frame : 1);
	capture_conf.skip_frames = 0;
	capture_conf.offset = -1;
	iAtomispObj_cap_main.xioctl(ATOMISP_IOC_S_CONT_CAPTURE_CONFIG, &capture_conf);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	control.value = 1;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	ext_control.value = 1;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	control.value = 5 + sdv_cap_num_frame;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	ext_control.value = 5 + sdv_cap_num_frame;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_VIEWFINDER;
	control.value = 1;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_VIEWFINDER;
	ext_control.value = 1;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	iAtomispObj_cap_main.set_format(sdv_cap_width, sdv_cap_height, sdv_cap_format);
	iAtomispObj_cap_main.init_device();
	iAtomispObj_cap_vf.set_format(sdv_cap_vf_width, sdv_cap_vf_height, sdv_cap_vf_format);
	iAtomispObj_cap_vf.init_device();

	iAtomispObj_main.set_format(main_width, main_height, main_format);
	iAtomispObj_main.init_device();
	iAtomispObj_vf.set_format(vf_width, vf_height, vf_format);
	iAtomispObj_vf.init_device();

	iAtomispObj_cap_main.init_buffers();
	iAtomispObj_cap_vf.init_buffers();
	iAtomispObj_main.init_buffers();
	iAtomispObj_vf.init_buffers();

	/* Configure test pattern. */
	control.id = V4L2_CID_TEST_PATTERN;
	control.value = test_pattern;
	iAtomispObj_main.xioctl(VIDIOC_S_CTRL, &control);

	flip_control(iAtomispObj_main, horizontal_flip, vertical_flip);

	/* start capture */
	iAtomispObj_main.start_capturing();
	iAtomispObj_vf.start_capturing();
	profile_start(&video_profile);
	iAtomispObj_main.mainloop(1);
	iAtomispObj_vf.mainloop(1);

	video_loop.loop = num_frame - 1;
	video_loop.main = &iAtomispObj_main;
	video_loop.vf = &iAtomispObj_vf;
	pthread_create(&tid1, NULL, frame_loop, &video_loop);

	if (sdv_cap_num_frame) {
		capture_loop.loop = sdv_cap_num_frame;
		capture_loop.main = &iAtomispObj_cap_main;
		capture_loop.vf = &iAtomispObj_cap_vf;
		iAtomispObj_cap_vf.start_capturing();
		profile_start(&capture_profile);
		iAtomispObj_cap_main.start_capturing();
		pthread_create(&tid2, NULL, frame_loop, &capture_loop);

		pthread_join(tid2, NULL);
		profile_end(&capture_profile);
		/* stop capture */
		iAtomispObj_cap_main.stop_capturing();
		iAtomispObj_cap_vf.stop_capturing();
	}

	pthread_join(tid1, NULL);
	profile_end(&video_profile);

	iAtomispObj_main.stop_capturing();
	iAtomispObj_vf.stop_capturing();

	/* Uninit device */
	iAtomispObj_cap_main.uninit_device();
	iAtomispObj_cap_vf.uninit_device();
	iAtomispObj_main.uninit_device();
	iAtomispObj_vf.uninit_device();

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	control.value = 0;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	ext_control.value = 0;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	control.value = 0;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	ext_control.value = 0;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	/* close device */
	iAtomispObj_cap_main.close_device();
	iAtomispObj_cap_vf.close_device();
	iAtomispObj_main.close_device();
	iAtomispObj_vf.close_device();
}


static void process_zsl_preview()
{
	struct v4l2_control control;
	struct v4l2_ext_controls controls;
	struct v4l2_ext_control ext_control;
	struct v4l2_streamparm parm;
	class CAtomispObj iAtomispObj_main(vf_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_cap_main(main_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	class CAtomispObj iAtomispObj_cap_vf(post_out_node[sub_stream], io, saveFile, cam_input, run_mode);
	pthread_t tid1, tid2;
	struct atomisp_loop preview_loop, capture_loop;
	struct atomisp_cont_capture_conf capture_conf;
	struct time_profile preview_profile, capture_profile;
	preview_profile.proc_name = "Preview";
	preview_profile.loop = num_frame;
	capture_profile.proc_name = "Capture+VF";
	capture_profile.loop = zsl_cap_num_frame;

	printf("ZSL preview mode: %dx%d, %s.\n", main_width, main_height,
	       format_to_str(main_format));
	printf("ZSL capture mode: %dx%d, %s, %dx%d, %s.\n", zsl_cap_width,
			zsl_cap_height, format_to_str(zsl_cap_format), zsl_cap_vf_width,
			zsl_cap_vf_height, format_to_str(zsl_cap_vf_format));

	/* open main output */
	iAtomispObj_main.open_device();
	iAtomispObj_cap_main.open_device();
	iAtomispObj_cap_vf.open_device();

	/* set ISP mode */
	memset(&parm, 0, sizeof(parm));
	parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	parm.parm.capture.capturemode = CI_MODE_PREVIEW;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_PARM, &parm);

	/* Init device */
	memset(&capture_conf, 0, sizeof(capture_conf));
	capture_conf.num_captures = (zsl_cap_num_frame ? zsl_cap_num_frame : 1);
	capture_conf.skip_frames = 0;
	capture_conf.offset = -1;
	iAtomispObj_cap_main.xioctl(ATOMISP_IOC_S_CONT_CAPTURE_CONFIG, &capture_conf);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	control.value = 1;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	ext_control.value = 1;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	control.value = 5 + zsl_cap_num_frame;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	ext_control.value = 5 + zsl_cap_num_frame;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_VIEWFINDER;
	control.value = 0; /* disable CVF */
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_VIEWFINDER;
	ext_control.value = 0; /* disable CVF */
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	iAtomispObj_cap_main.set_format(zsl_cap_width, zsl_cap_height, zsl_cap_format);
	iAtomispObj_cap_main.init_device();
	iAtomispObj_cap_vf.set_format(zsl_cap_vf_width, zsl_cap_vf_height, zsl_cap_vf_format);
	iAtomispObj_cap_vf.init_device();

	iAtomispObj_main.set_format(main_width, main_height, main_format);
	iAtomispObj_main.init_device();

	iAtomispObj_cap_main.init_buffers();
	iAtomispObj_cap_vf.init_buffers();
	iAtomispObj_main.init_buffers();

	/* Configure test pattern. */
	control.id = V4L2_CID_TEST_PATTERN;
	control.value = test_pattern;
	iAtomispObj_main.xioctl(VIDIOC_S_CTRL, &control);

	flip_control(iAtomispObj_main, horizontal_flip, vertical_flip);

	/* start capture */
	iAtomispObj_main.start_capturing();
	profile_start(&preview_profile);
	iAtomispObj_main.mainloop(1);

	preview_loop.loop = num_frame - 1;
	preview_loop.main = &iAtomispObj_main;
	preview_loop.vf = NULL;
	pthread_create(&tid1, NULL, frame_loop, &preview_loop);

	if (zsl_cap_num_frame) {
		capture_loop.loop = zsl_cap_num_frame;
		capture_loop.main = &iAtomispObj_cap_main;
		capture_loop.vf = &iAtomispObj_cap_vf;
		iAtomispObj_cap_vf.start_capturing();
		profile_start(&capture_profile);
		iAtomispObj_cap_main.start_capturing();
		pthread_create(&tid2, NULL, frame_loop, &capture_loop);

		pthread_join(tid2, NULL);
		profile_end(&capture_profile);
		/* stop capture */
		iAtomispObj_cap_main.stop_capturing();
		iAtomispObj_cap_vf.stop_capturing();
	}

	pthread_join(tid1, NULL);
	profile_end(&preview_profile);

	iAtomispObj_main.stop_capturing();

	/* Uninit device */
	iAtomispObj_cap_main.uninit_device();
	iAtomispObj_cap_vf.uninit_device();
	iAtomispObj_main.uninit_device();

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	control.value = 0;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_MODE;
	ext_control.value = 0;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	memset(&control, 0, sizeof(control));
	memset(&controls, 0, sizeof(controls));
	memset(&ext_control, 0, sizeof(ext_control));
	control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	control.value = 0;
	controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
	controls.count = 1;
	controls.controls = &ext_control;
	ext_control.id = V4L2_CID_ATOMISP_CONTINUOUS_RAW_BUFFER_SIZE;
	ext_control.value = 0;
	iAtomispObj_cap_main.xioctl(VIDIOC_S_EXT_CTRLS, &controls);
	iAtomispObj_cap_main.xioctl(VIDIOC_S_CTRL, &controls);

	/* close device */
	iAtomispObj_cap_main.close_device();
	iAtomispObj_cap_vf.close_device();
	iAtomispObj_main.close_device();
}


static int dbg_h_v_blank(void)
{
	int fd;
	char subdev[64];
	struct v4l2_control control;
	struct v4l2_ext_controls ext_controls;
	struct v4l2_ext_control ext_control;
	unsigned long _pixel_rate;
	int _h_blank;
	int _v_blank;
	int ret = 0;

	snprintf(subdev, 64, "/dev/v4l-subdev%d", subdev_index);
	fd = open(subdev, O_RDWR /* required */  | O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
			subdev, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}
	/* Get H Blank */
	control.id = V4L2_CID_HBLANK;
	do
		ret = ioctl(fd, VIDIOC_G_CTRL, &control);
	while (-1 == ret && EINTR == errno);
	if (!ret) {
		_h_blank = control.value;
	} else {
		fprintf(stderr, "Failed to get H Blank: %d, %s\n",
			errno, strerror(errno));
		goto end;
	}

	/* Get V Blank */
	control.id = V4L2_CID_VBLANK;
	do
		ret = ioctl(fd, VIDIOC_G_CTRL, &control);
	while (-1 == ret && EINTR == errno);
	if (!ret) {
		_v_blank = control.value;
	} else {
		fprintf(stderr, "Failed to get V Blank: %d, %s\n",
			errno, strerror(errno));
		goto end;
	}
	/* Get Pixel Rate */
	ext_control.id = V4L2_CID_PIXEL_RATE;
	ext_controls.ctrl_class = V4L2_CTRL_ID2CLASS(ext_control.id);
	ext_controls.count = 1;
	ext_controls.controls = &ext_control;
	do
		ret = ioctl(fd, VIDIOC_G_EXT_CTRLS, &ext_controls);
	while (-1 == ret && EINTR == errno);
	if (!ret) {
		_pixel_rate = ext_control.value64;
	} else {
		printf("SUBDEV IOCTL:%d.\n", ret);
		fprintf(stderr, "Failed to get Pixel rate: %d, %s\n",
			errno, strerror(errno));
		goto end;
	}
	/* Calculate and Display H/V Blank time */
	printf("Get HBlank:%d, VBlank:%d, Pixel Rate: %ld.\n",
	       _h_blank, _v_blank, _pixel_rate);
	/* Set New H/V Blank time if avaiable */
	/* Set H Blank */
	if (h_blank >= 0) {
		control.id = V4L2_CID_HBLANK;
		control.value = h_blank;
		do
			ret = ioctl(fd, VIDIOC_S_CTRL, &control);
		while (-1 == ret && EINTR == errno);
		if (!ret) {
			if (!h_blank)
				printf("Reset to original h_blank.\n");
			else
				printf("Set New H Blank:%d.\n", control.value);
		} else {
			fprintf(stderr, "Failed to set H Blank: %d, %s\n",
				errno, strerror(errno));
			goto end;
		}
	}
	/* Set V Blank */
	if (v_blank >= 0) {
		control.id = V4L2_CID_VBLANK;
		control.value = v_blank;
		do
			ret = ioctl(fd, VIDIOC_S_CTRL, &control);
		while (-1 == ret && EINTR == errno);
		if (!ret) {
			if (!v_blank)
				printf("Reset to original v_blank.\n");
			else
				printf("Set New V Blank:%d.\n", control.value);
		} else {
			fprintf(stderr, "Failed to set V Blank: %d, %s\n",
				errno, strerror(errno));
			goto end;
		}
	}

end:
	close(fd);
	return ret;

}

char *format_to_str(int format)
{
	switch (format) {
	case V4L2_PIX_FMT_YUV420:
		return (char*)"yuv420";
	case V4L2_PIX_FMT_YVU420:
		return (char*)"yv12";
	case V4L2_PIX_FMT_YUV422P:
		return (char*)"yuv422";
	case V4L2_PIX_FMT_YUV444:
		return (char*)"yuv444";
	case V4L2_PIX_FMT_NV12:
		return (char*)"nv12";
	case V4L2_PIX_FMT_NV21:
		return (char*)"nv21";
	case V4L2_PIX_FMT_NV16:
		return (char*)"nv16";
	case V4L2_PIX_FMT_NV61:
		return (char*)"nv61";
	case V4L2_PIX_FMT_UYVY:
		return (char*)"uyvy";
	case V4L2_PIX_FMT_YUYV:
		return (char*)"yuyv";
	case V4L2_PIX_FMT_RGB565:
		return (char*)"rgb565";
	case V4L2_PIX_FMT_RGB24:
		return (char*)"rgb24";
	case V4L2_PIX_FMT_RGB32:
		return (char*)"rgb32";
	case V4L2_PIX_FMT_SRGGB8:
		return (char*)"rggb8";
	case V4L2_PIX_FMT_SGRBG8:
		return (char*)"grbg8";
	case V4L2_PIX_FMT_SGBRG8:
		return (char*)"gbrg8";
	case V4L2_PIX_FMT_SBGGR8:
		return (char*)"bggr8";
	case V4L2_PIX_FMT_SRGGB10:
		return (char*)"rggb10";
	case V4L2_PIX_FMT_SGRBG10:
		return (char*)"grbg10";
	case V4L2_PIX_FMT_SGBRG10:
		return (char*)"gbrg10";
	case V4L2_PIX_FMT_SBGGR10:
		return (char*)"bggr10";
	default:
		return (char*)"yuv420";
	}
}

unsigned char format_to_bytes(int format)
{
	switch (format) {
	case V4L2_PIX_FMT_YUV420:
		return 1;
	case V4L2_PIX_FMT_YVU420:
		return 1;
	case V4L2_PIX_FMT_YUV422P:
		return 1;
	case V4L2_PIX_FMT_YUV444:
		return 3;
	case V4L2_PIX_FMT_NV12:
		return 1;
	case V4L2_PIX_FMT_NV21:
		return 1;
	case V4L2_PIX_FMT_NV16:
		return 1;
	case V4L2_PIX_FMT_NV61:
		return 1;
	case V4L2_PIX_FMT_UYVY:
		return 2;
	case V4L2_PIX_FMT_YUYV:
		return 2;
	case V4L2_PIX_FMT_RGB565:
		return 2;
	case V4L2_PIX_FMT_RGB24:
		return 3;
	case V4L2_PIX_FMT_RGB32:
		return 4;
	case V4L2_PIX_FMT_SBGGR8:
		return 1;
	case V4L2_PIX_FMT_SRGGB10:
		return 2;
	case V4L2_PIX_FMT_SGRBG10:
		return 2;
	case V4L2_PIX_FMT_SGBRG10:
		return 2;
	case V4L2_PIX_FMT_SBGGR10:
		return 2;
	default:
		return 1;
	}
}
