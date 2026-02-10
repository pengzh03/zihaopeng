CC = gcc
CXX = g++
CFLAGS = -c -Wall -g -Iinclude
CXXFLAGS = -c -pipe -std=c++11 -02 -Wall -W -D_REENTRANT -Iinclude -Ilib
CXXDEBUGFLAGS = -g -O0 -c -pipe -std=c++11 -Wall -W -D_REENTRANT -Iinclude -Ilib
LDFLAGS = -pthread

BUILD_DIR = build
SOURCE_DIR = src
TARGET = $(BUILD_DIR)router
OBJS = $(BUILD_DIR)/main.o $(BUILD_DIR)/builder.o $(BUILD_DIR)/ioutils.o $(BUILD_DIR)/geometry.o $(BUILD_DIR)/kmean.o $(BUILD_DIR)/direct_route.o $(BUILD_DIR)/layer_assignment.o $(BUILD_DIR)/build_exits.o $(BUILD_DIR)/a_star_direct.o $(BUILD_DIR)/a_star.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) ${LDFLAGS} -o $@

$(BUILD_DIR)/main.o: $(SOURCE_DIR)/main.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/main.cpp -o $@

$(BUILD_DIR)/builder.o: $(SOURCE_DIR)/builder.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/builder.cpp -o $@

$(BUILD_DIR)/ioutils.o: $(SOURCE_DIR)/ioutils.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/ioutils.cpp -o $@

$(BUILD_DIR)/a_star.o: $(SOURCE_DIR)/a_star.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/a_star.cpp -o $@

$(BUILD_DIR)/geometry.o: $(SOURCE_DIR)/geometry.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/geometry.cpp -o $@

$(BUILD_DIR)/kmean.o: $(SOURCE_DIR)/kmean.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/kmean.cpp -o $@

$(BUILD_DIR)/direct_route.o: $(SOURCE_DIR)/direct_route.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/direct_route.cpp -o $@

$(BUILD_DIR)/layer_assignment.o: $(SOURCE_DIR)/layer_assignment.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/layer_assignment.cpp -o $@

$(BUILD_DIR)/build_exits.o: $(SOURCE_DIR)/build_exits.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/build_exits.cpp -o $@

$(BUILD_DIR)/a_star_direct.o: $(SOURCE_DIR)/a_star_direct.cpp | $(BUILD_DIR)
	$(CXX) $(CXXDEBUGFLAGS) $(SOURCE_DIR)/a_star_direct.cpp -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)