# ---------------------------------------------------------------
# Makefile for ROS 2 Jazzy RosAria Node
# ---------------------------------------------------------------

CXX       := g++
CXXFLAGS  := -std=c++17 -Wall -Wextra -O2 -fPIC

# ROS 2 include and library paths (colcon setup file must be sourced)
ROS2_CFLAGS := $(shell pkg-config --cflags rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)
ROS2_LIBS   := $(shell pkg-config --libs rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

# Boost
BOOST_LIBS := -lboost_system -lboost_thread

# ARIA installation
ARIA_DIR ?= /usr/local/Aria
ARIA_INC := -I$(ARIA_DIR)/include
ARIA_LIBS := -L$(ARIA_DIR)/lib -lAria -lpthread -ldl -lrt

# Output
TARGET = RosAria
SRC = RosAria.cpp LaserPublisher.cpp
OBJ = $(SRC:.cpp=.o)

# ---------------------------------------------------------------
# Rules
# ---------------------------------------------------------------

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) $(OBJ) -o $@ $(ROS2_LIBS) $(ARIA_LIBS) $(BOOST_LIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(ROS2_CFLAGS) $(ARIA_INC) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

install:
	install -m 755 $(TARGET) /usr/local/bin/

run: $(TARGET)
	./$(TARGET)

info:
	@echo "Compiler: $(CXX)"
	@echo "CXXFLAGS: $(CXXFLAGS)"
	@echo "ARIA_DIR: $(ARIA_DIR)"
	@echo "ROS2_CFLAGS: $(ROS2_CFLAGS)"
	@echo "ROS2_LIBS: $(ROS2_LIBS)"

.PHONY: all clean install run info
