# Makefile for list_cameras

CXX      := g++
CXXFLAGS := -std=c++17 -Wall -Wextra

TARGET   := list_cameras
SRC      := list_cameras.cpp

# Prefer pkg-config if libcamera is discoverable; otherwise fall back to
# the ROS Humble vendored paths.
PKG := $(shell pkg-config --exists libcamera && echo yes)
ifeq ($(PKG),yes)
  INCLUDES := $(shell pkg-config --cflags libcamera)
  LDFLAGS  :=
  LDLIBS   := $(shell pkg-config --libs libcamera)
else
  INCLUDES := -I/opt/ros/humble/include/libcamera
  LDFLAGS  := -L/opt/ros/humble/lib
  LDLIBS   := -lcamera -lcamera-base
endif

.PHONY: all list-cameras clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET) $(LDFLAGS) $(LDLIBS)

list-cameras: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
