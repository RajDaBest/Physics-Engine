CC=gcc
CFLAGS=-fPIC -Wall -Wextra -O2
LDFLAGS=-shared

SRC_DIR=.
BUILD_DIR=build

SRCS=physics_plugin.c
OBJS=$(SRCS:%.c=$(BUILD_DIR)/%.o)
TARGET=libphysics_plugin.so

.PHONY: all clean

all: $(BUILD_DIR)/$(TARGET)

$(BUILD_DIR)/$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)