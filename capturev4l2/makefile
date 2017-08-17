CC	= g++
LD	= $(CC)

SRC	= $(wildcard *.cpp)
OBJS	= $(SRC:.cpp=.o)

%.o: %.cpp
	$(CC) -c -pthread -o $@ $<

capture_v4l2: $(OBJS)
	$(LD) -pthread -o $@ $(OBJS)

clean:
	rm -rf $(OBJS) capture_v4l2
