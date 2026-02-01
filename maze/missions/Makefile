# missions/Makefile
# Builds the mission dashboard tool.
#
# Default build uses hiredis (Redis client library).
# To build without Redis dependency:
#   make NO_REDIS=1

CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11

TARGET := mission_dashboard
SRC := mission_dashboard.c

ifeq ($(NO_REDIS),1)
  CFLAGS += -DNO_REDIS
  LDLIBS :=
else
  LDLIBS := -lhiredis
endif

all: $(TARGET)

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(TARGET) $(LDLIBS)

clean:
	rm -f $(TARGET)
