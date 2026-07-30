CC := gcc

TARGET := physim

SRC_DIR := src
INC_DIR := include
OBJ_DIR := build
BIN_DIR := bin

SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC))
DEP := $(OBJ:.o=.d)

CPPFLAGS := -I$(INC_DIR) $(shell pkg-config --cflags raylib)
CFLAGS := -std=c11 -g -O0 -Wall -Wextra -Wpedantic -Wconversion -Wshadow -MMD -MP
LDLIBS := $(shell pkg-config --libs raylib)

.PHONY: all run clean rebuild

all: $(BIN_DIR)/$(TARGET)

$(BIN_DIR)/$(TARGET): $(OBJ) | $(BIN_DIR)
	$(CC) $(OBJ) -o $@ $(LDLIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

run: all
	./$(BIN_DIR)/$(TARGET)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

rebuild: clean all

-include $(DEP)