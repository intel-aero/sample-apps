#ifndef _V4L2_OBJ
#define _V4L2_OBJ

#include <stdio.h>
#include <string.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

typedef enum {
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
} io_method;

class CV4l2Obj {

private:
	char *m_devName;
	io_method m_io;
	struct Buffer {
		void *start;
		size_t length;
	};
	struct Buffer *m_Buffers;
	unsigned int m_nBuffers;

	int m_frameCnt;

	int m_saveFile;
protected:
	int m_width;
	int m_padded_width;
	int m_height;
	int m_format;
	int m_sizeimage;
public:
	CV4l2Obj(char *devName, io_method io, int saveFile);
	int stop_capturing(void);
	int start_capturing(void);
	int uninit_device(void);
	int init_device(void);
	int init_buffers(void);
	int open_device(void);
	int close_device(void);
	int mainloop(unsigned int count);
	int xioctl(int request, void *arg);

	int m_fd;
private:
	int init_read(unsigned int buffer_size);
	int init_mmap(void);
	int init_userp(unsigned int buffer_size);
protected:
	int read_frame(void);
	int process_image(const void *p, int size);

	void errno_exit(const char *s);
};
#endif
