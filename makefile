CC = gcc

# Apple Clang ships without the OpenMP runtime; libomp must be installed via
# Homebrew ('brew install libomp'). Detect the Homebrew prefix at build time so
# the makefile works on both Apple Silicon (/opt/homebrew) and Intel (/usr/local).
HOMEBREW_PREFIX := $(shell brew --prefix 2>/dev/null || echo /usr/local)
LIBOMP := $(HOMEBREW_PREFIX)/opt/libomp

CFLAGS  = -Wall -O3 -std=c11 -Iheaders \
           -Xclang -fopenmp -I$(LIBOMP)/include
LDFLAGS = -lm -L$(LIBOMP)/lib -lomp

SRC = $(wildcard src/**/*.c) $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)

TARGET = denoise

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)