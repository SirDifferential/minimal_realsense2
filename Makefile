CC=gcc
CFLAGS=-I/home/gekko/librealsense/include
LDFLAGS=-lSDL2 -L/home/gekko/librealsense2/build -lrealsense2
SOURCES=main.c
OBJECTS=$(SOURCES:.c=.o)
EXECUTABLE=minimal_realsense2

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(CFLAGS) -o $(EXECUTABLE) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $<

clean:
	rm *.o

