# 编译器指定
CC = gcc
CXX = g++
# C编译选项
CFLAGS = -c -Wall -O3 -g -march=native -mtune=native -flto -fprefetch-loop-arrays -fopenmp -Iinclude  
# C++发布版本选项
CXXFLAGS = -c -pipe -std=c++17 -O3 -g -march=native -mtune=native -flto -funroll-loops -ftree-vectorize -fprefetch-loop-arrays -ffast-math -Wall -W -D_REENTRANT -fopenmp -Iinclude -Ilib
# C++调试版本选项（仅调试时用）
CXXDEBUGFLAGS = -g -O0 -c -pipe -std=c++11 -Wall -W -D_REENTRANT -Iinclude -Ilib
# 链接选项
LDFLAGS = -pthread -O3 -g -march=native -flto -fopenmp

BUILD_DIR = build
SOURCE_DIR = src
TARGET = $(BUILD_DIR)/router
OBJS = $(BUILD_DIR)/main.o $(BUILD_DIR)/builder.o $(BUILD_DIR)/ioutils.o $(BUILD_DIR)/geometry.o $(BUILD_DIR)/kmean.o $(BUILD_DIR)/direct_route.o $(BUILD_DIR)/layer_assignment.o $(BUILD_DIR)/build_exits.o $(BUILD_DIR)/a_star_direct.o $(BUILD_DIR)/a_star.o

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) ${LDFLAGS} -o $@

# 以下所有编译规则：从CXXDEBUGFLAGS 改为 CXXFLAGS（让-O3等优化生效，核心修改）# 修改：CXXDEBUGFLAGS → CXXFLAGS
$(BUILD_DIR)/main.o: $(SOURCE_DIR)/main.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/main.cpp -o $@

$(BUILD_DIR)/builder.o: $(SOURCE_DIR)/builder.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/builder.cpp -o $@

$(BUILD_DIR)/ioutils.o: $(SOURCE_DIR)/ioutils.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/ioutils.cpp -o $@

$(BUILD_DIR)/a_star.o: $(SOURCE_DIR)/a_star.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/a_star.cpp -o $@

$(BUILD_DIR)/geometry.o: $(SOURCE_DIR)/geometry.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/geometry.cpp -o $@

$(BUILD_DIR)/kmean.o: $(SOURCE_DIR)/kmean.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/kmean.cpp -o $@ 

$(BUILD_DIR)/direct_route.o: $(SOURCE_DIR)/direct_route.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/direct_route.cpp -o $@

$(BUILD_DIR)/layer_assignment.o: $(SOURCE_DIR)/layer_assignment.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/layer_assignment.cpp -o $@

$(BUILD_DIR)/build_exits.o: $(SOURCE_DIR)/build_exits.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/build_exits.cpp -o $@

$(BUILD_DIR)/a_star_direct.o: $(SOURCE_DIR)/a_star_direct.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(SOURCE_DIR)/a_star_direct.cpp -o $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)