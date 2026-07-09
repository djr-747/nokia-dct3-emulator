CC      ?= gcc
CFLAGS  ?= -O2 -Wall -Wextra -std=c11
LDLIBS   = -lregina

nokix: nokix.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

clean:
	rm -f nokix

.PHONY: clean
