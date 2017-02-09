#include "atomisp_obj.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <linux/videodev2.h>

CAtomispObj::CAtomispObj(char *devName, io_method io, int saveFile, int cameraId, int ispMode): CV4l2Obj(devName, io, saveFile), m_cameraId(cameraId), m_ispMode(ispMode)
{

}

int CAtomispObj::open_device(bool set_input)
{
	CV4l2Obj::open_device();

	/* set ISP mode */
	struct v4l2_streamparm parm;

	/* set input */
	if (set_input && -1 == xioctl(VIDIOC_S_INPUT, &m_cameraId))
		errno_exit("VIDIOC_S_INPUT");

        CLEAR(parm);
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.capturemode = m_ispMode;
        if (-1 == xioctl(VIDIOC_S_PARM, &parm))
                errno_exit("VIDIOC_S_PARM");

	return 0;
}

void CAtomispObj::set_format(int width, int height, int format)
{
	m_width = width;
	m_height = height;
	m_format = format;
}
