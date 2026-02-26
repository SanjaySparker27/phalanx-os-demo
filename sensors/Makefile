# Unified Sensor Library Makefile
# Builds all sensor drivers and utilities

# Compiler settings
CC = gcc
CXX = g++
CFLAGS = -Wall -Wextra -O2 -fPIC -std=c11
CXXFLAGS = -Wall -Wextra -O2 -fPIC -std=c++17
INCLUDES = -I. -I/usr/include/glib-2.0 -I/usr/lib/glib-2.0/include
LDFLAGS = -shared -lpthread -lm -lgstapp-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0

# Directories
SRCDIR = .
COREDIR = core
CAMDIR = cameras
GNSSDIR = gnss
IMUDIR = imu
LIDARDIR = lidar
SATCOMDIR = satcom
TELEMDIR = telemetry
ENVDIR = environment
UTILSDIR = utils
TESTDIR = tests
BUILDDIR = build
LIBDIR = lib

# Target library
TARGET = $(LIBDIR)/libsensors.so
STATIC_TARGET = $(LIBDIR)/libsensors.a

# Source files
CORE_SRCS = $(COREDIR)/sensor_base.c $(COREDIR)/sensor_manager.c
CAM_SRCS = $(CAMDIR)/gstreamer_pipeline.c
# Individual driver implementations would be added here

ALL_SRCS = $(CORE_SRCS) $(CAM_SRCS)
OBJS = $(patsubst %.c,$(BUILDDIR)/%.o,$(notdir $(ALL_SRCS)))

# Header files
HEADERS = $(wildcard $(COREDIR)/*.h) \
          $(wildcard $(CAMDIR)/*.h) \
          $(wildcard $(GNSSDIR)/*.h) \
          $(wildcard $(IMUDIR)/*.h) \
          $(wildcard $(LIDARDIR)/*.h) \
          $(wildcard $(SATCOMDIR)/*.h) \
          $(wildcard $(TELEMDIR)/*.h) \
          $(wildcard $(ENVDIR)/*.h) \
          $(wildcard $(UTILSDIR)/*.h) \
          sensors.h

.PHONY: all clean install test dirs

all: dirs $(TARGET) $(STATIC_TARGET)

dirs:
	@mkdir -p $(BUILDDIR) $(LIBDIR) $(TESTDIR)

# Pattern rules
$(BUILDDIR)/%.o: $(COREDIR)/%.c $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILDDIR)/%.o: $(CAMDIR)/%.c $(HEADERS)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Shared library
$(TARGET): $(OBJS)
	$(CC) -shared -o $@ $^ $(LDFLAGS)
	@echo "Built shared library: $@"

# Static library
$(STATIC_TARGET): $(OBJS)
	ar rcs $@ $^
	@echo "Built static library: $@"

# Test programs
test: dirs $(TARGET) $(TESTDIR)/sensor_test

$(TESTDIR)/sensor_test: $(TESTDIR)/test_sensor_manager.c $(TARGET)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $< -L$(LIBDIR) -lsensors $(LDFLAGS) -Wl,-rpath,'$$ORIGIN/../lib'

# Install to system
install: all
	install -d /usr/local/lib
	install -d /usr/local/include/sensors
	install -m 644 $(TARGET) /usr/local/lib/
	install -m 644 $(STATIC_TARGET) /usr/local/lib/
	install -m 644 sensors.h /usr/local/include/sensors/
	install -m 644 $(COREDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(CAMDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(GNSSDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(IMUDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(LIDARDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(SATCOMDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(TELEMDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(ENVDIR)/*.h /usr/local/include/sensors/
	install -m 644 $(UTILSDIR)/*.h /usr/local/include/sensors/
	ldconfig
	@echo "Installed to /usr/local"

# Uninstall
uninstall:
	rm -f /usr/local/lib/libsensors.so
	rm -f /usr/local/lib/libsensors.a
	rm -rf /usr/local/include/sensors
	ldconfig

# Clean build artifacts
clean:
	rm -rf $(BUILDDIR) $(LIBDIR) $(TESTDIR)/*.o $(TESTDIR)/sensor_test

# Format code
format:
	find . -name "*.c" -o -name "*.h" | xargs clang-format -i

# Static analysis
analyze:
	scan-build make clean all

# Documentation
docs:
	doxygen Doxyfile 2>/dev/null || echo "Doxygen not installed"

# Debug build
debug: CFLAGS = -Wall -Wextra -g -O0 -fPIC -std=c11 -DDEBUG
debug: all

# Release build with optimizations
release: CFLAGS = -Wall -Wextra -O3 -fPIC -std=c11 -DNDEBUG -ffast-math
release: all

# Print help
help:
	@echo "Unified Sensor Library Build System"
	@echo "==================================="
	@echo "Targets:"
	@echo "  all      - Build shared and static libraries"
	@echo "  test     - Build test programs"
	@echo "  install  - Install to /usr/local"
	@echo "  clean    - Remove build artifacts"
	@echo "  format   - Format source code"
	@echo "  debug    - Debug build"
	@echo "  release  - Optimized release build"
