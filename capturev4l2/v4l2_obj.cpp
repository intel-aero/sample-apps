#include "v4l2_obj.h"
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

#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

#include <linux/videodev2.h>

extern char *format_to_str(int format);
extern unsigned char format_to_bytes(int format);

CV4l2Obj::CV4l2Obj (char *devName, io_method io, int saveFile):
	m_devName(devName), m_io(io), m_saveFile(saveFile)
{
	m_width = 640;
	m_height = 480;
	m_format = V4L2_PIX_FMT_YUV420;
	m_frameCnt = 0;
	// m_saveFile = 0;
}

int CV4l2Obj::init_device()
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;

	if (-1 == xioctl(VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n", m_devName);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n", m_devName);
		exit(EXIT_FAILURE);
	}

	switch (m_io) {
	case IO_METHOD_READ:
		if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf(stderr, "%s does not support read i/o\n",
				m_devName);
			exit(EXIT_FAILURE);
		}

		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf(stderr, "%s does not support streaming i/o\n",
				m_devName);
			exit(EXIT_FAILURE);
		}

		break;
	}

	/* Select video input, video standard and tune here. */

	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect;	/* reset to default */

		if (-1 == xioctl(VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	CLEAR(fmt);

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = m_width;
	fmt.fmt.pix.height = m_height;
	fmt.fmt.pix.pixelformat = m_format;
	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

	if (-1 == xioctl(VIDIOC_S_FMT, &fmt))
		errno_exit("VIDIOC_S_FMT");

	/* Note VIDIOC_S_FMT may change width and height. */
	m_width = fmt.fmt.pix.width;
	m_height = fmt.fmt.pix.height;
	m_format = fmt.fmt.pix.pixelformat;
	m_padded_width = fmt.fmt.pix.bytesperline / format_to_bytes(m_format);
	m_sizeimage = fmt.fmt.pix.sizeimage;

	printf("m_width %d, m_height %d, m_sizeimage %d, m_padded_width:%d, bytesperline %d\n",
			m_width, m_height, m_sizeimage, m_padded_width, fmt.fmt.pix.bytesperline);

	return 0;
}

int CV4l2Obj::init_buffers()
{
	switch (m_io) {
	case IO_METHOD_READ:
		return init_read(m_sizeimage);

	case IO_METHOD_MMAP:
		return init_mmap();

	case IO_METHOD_USERPTR:
		return init_userp(m_sizeimage);
	}

	return 0;
}

int CV4l2Obj::uninit_device(void)
{
	unsigned int i;

	switch (m_io) {
	case IO_METHOD_READ:
		free(m_Buffers[0].start);
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < m_nBuffers; ++i)
			if (-1 == munmap(m_Buffers[i].start, m_Buffers[i].length))
				errno_exit("munmap");
		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < m_nBuffers; ++i)
			free(m_Buffers[i].start);
		break;
	}

	free(m_Buffers);

	return 0;
}

int CV4l2Obj::open_device()
{
	struct stat st;

	if (-1 == stat(m_devName, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
			m_devName, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", m_devName);
		exit(EXIT_FAILURE);
	}

	m_fd = open(m_devName, O_RDWR /* required */  | O_NONBLOCK, 0);

	if (-1 == m_fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
			m_devName, errno, strerror(errno));
		exit(EXIT_FAILURE);
	}

	return 0;
}

int CV4l2Obj::close_device()
{
	if (-1 == close(m_fd))
		errno_exit("close");

	m_fd = -1;

	return 0;
}

int CV4l2Obj::stop_capturing(void)
{
	enum v4l2_buf_type type;

	switch (m_io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl(VIDIOC_STREAMOFF, &type))
			errno_exit("VIDIOC_STREAMOFF");

		break;
	}
	return 0;
}

int CV4l2Obj::start_capturing(void)
{
	unsigned int i;
	enum v4l2_buf_type type;

	switch (m_io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < m_nBuffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);

			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = i;

			if (-1 == xioctl(VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF");
		}

		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl(VIDIOC_STREAMON, &type))
			errno_exit("VIDIOC_STREAMON");

		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < m_nBuffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);

			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;
			buf.index = i;
			buf.m.userptr = (unsigned long)m_Buffers[i].start;
			buf.length = m_Buffers[i].length;

			if (-1 == xioctl(VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF");
		}

		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

		if (-1 == xioctl(VIDIOC_STREAMON, &type))
			errno_exit("VIDIOC_STREAMON");

		break;
	}
	return 0;
}

int CV4l2Obj::mainloop(unsigned int count)
{
	while (count-- > 0) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(m_fd, &fds);

			/* Timeout. */
			tv.tv_sec = 4;
			tv.tv_usec = 0;

			r = select(m_fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;

				errno_exit("select");
			}

			if (0 == r) {
				fprintf(stderr, "select timeout\n");
				exit(EXIT_FAILURE);
			}

			if (read_frame())
				break;

			/* EAGAIN - continue select loop. */
		}
	}
	return 0;
}

int CV4l2Obj::init_read(unsigned int buffer_size)
{
	m_Buffers = (struct Buffer*)calloc(1, sizeof(*m_Buffers));

	if (!m_Buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	m_Buffers[0].length = buffer_size;
	m_Buffers[0].start = malloc(buffer_size);

	if (!m_Buffers[0].start) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}
	return 0;
}

int CV4l2Obj::init_mmap(void)
{
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
				"memory mapping\n", m_devName);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", m_devName);
		exit(EXIT_FAILURE);
	}

	m_Buffers = (struct Buffer*)calloc(req.count, sizeof(*m_Buffers));

	if (!m_Buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (m_nBuffers = 0; m_nBuffers < req.count; ++m_nBuffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = m_nBuffers;

		if (-1 == xioctl(VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");

		m_Buffers[m_nBuffers].length = buf.length;
		m_Buffers[m_nBuffers].start = mmap(NULL /* start anywhere */ ,
						buf.length,
						PROT_READ | PROT_WRITE
						/* required */ ,
						MAP_SHARED /* recommended */ ,
						m_fd, buf.m.offset);

		if (MAP_FAILED == m_Buffers[m_nBuffers].start)
			errno_exit("mmap");
	}
	return 0;
}

int CV4l2Obj::init_userp(unsigned int buffer_size)
{
	struct v4l2_requestbuffers req;
	unsigned int page_size;

	page_size = getpagesize();
	buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

	CLEAR(req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
				"user pointer i/o\n", m_devName);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	m_Buffers = (struct Buffer*)calloc(4, sizeof(*m_Buffers));

	if (!m_Buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (m_nBuffers = 0; m_nBuffers < 4; ++m_nBuffers) {
		m_Buffers[m_nBuffers].length = buffer_size;
		m_Buffers[m_nBuffers].start = memalign( /* boundary */ page_size,
						    buffer_size);

		if (!m_Buffers[m_nBuffers].start) {
			fprintf(stderr, "Out of memory\n");
			exit(EXIT_FAILURE);
		}
	}
	return 0;
}

int CV4l2Obj::xioctl(int request, void *arg)
{
	int r;

	do
		r = ioctl(m_fd, request, arg);
	while (-1 == r && EINTR == errno);

	return r;
}

int CV4l2Obj::read_frame(void)
{
	struct v4l2_buffer buf;
	unsigned int i;

	switch (m_io) {
	case IO_METHOD_READ:
		if (-1 == read(m_fd, m_Buffers[0].start, m_Buffers[0].length)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit("read");
			}
		}

		process_image(m_Buffers[0].start, m_Buffers[0].length);

		break;

	case IO_METHOD_MMAP:
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl(VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}

		assert(buf.index < m_nBuffers);

		process_image(m_Buffers[buf.index].start, m_Buffers[buf.index].length);

		if (-1 == xioctl(VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");

		break;

	case IO_METHOD_USERPTR:
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;

		if (-1 == xioctl(VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}

		for (i = 0; i < m_nBuffers; ++i)
			if (buf.m.userptr == (unsigned long)m_Buffers[i].start
			    && buf.length == m_Buffers[i].length)
				break;

		assert(i < m_nBuffers);

		process_image((void *)buf.m.userptr, buf.length);

		if (-1 == xioctl(VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");

		break;
	}

	return 1;
}

int CV4l2Obj::process_image(const void *p, int size)
{
	if (m_saveFile) {
		ostringstream sFileName;
		ofstream fp;
		char *buf = (char *)p;
		char *s_format = format_to_str(m_format);;
		char *prefix = strstr(m_devName, "video");

		if (!buf) {
			cout << "Invalid buffer address!" << endl;
			exit(0);
		}

		sFileName << "Image-" << prefix << "-" << m_padded_width << "x" << m_height << "-" << m_frameCnt << "." << s_format;
		cout << "Saving file: " << sFileName.str().c_str() << endl;

		fp.open(sFileName.str().c_str());

		if (!fp) {
			cout << "Open file: " << sFileName.str().c_str() << " failed" << endl;
			exit(0);
		}

		fp.write(buf, size);

		fp.close();
	}

	m_frameCnt++;
	return 0;
}

void CV4l2Obj::errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));

	exit(EXIT_FAILURE);
}
