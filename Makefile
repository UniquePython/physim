CC := gcc

CFLAGS := -std=c11 -Wall -Wextra
CPPFLAGS := $(shell pkg-config --cflags raylib)
LDLIBS := $(shell pkg-config --libs raylib)

physim: main.c | build
	$(CC) $(CPPFLAGS) $(CFLAGS) $< -o build/$@ $(LDLIBS)

build:
	mkdir -p build