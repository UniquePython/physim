CC := gcc

CFLAGS := -std=c11 -Wall -Wextra $(shell pkg-config --cflags raylib)
LDLIBS := $(shell pkg-config --libs raylib)

physim: main.c
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)