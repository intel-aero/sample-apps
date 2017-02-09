#ifndef _ATOMISP_OBJ
#define _ATOMISP_OBJ

#include "v4l2_obj.h"

class CAtomispObj: public CV4l2Obj {
private:
	int m_cameraId;
	int m_ispMode;
public:
	CAtomispObj(char *devName, io_method io, int saveFile, int cameraId, int ispMode);
	int open_device(bool set_input = true);
	void set_format(int width, int height, int format);
};

#endif
