Simple app that demostrates optical flow using Aero RTF's downward facing camera.

You can build this directly on Aero compute board. Use scp and copy aero_flow_v4l2.cpp to Aero and compile it following command:

	g++ aero_flow_v4l2.cpp -o aero_flow_v4l2 `pkg-config --cflags --libs opencv`


To run, enter following command.

	./aero_flow_v4l2 -d /dev/video2 -i 1 -u


