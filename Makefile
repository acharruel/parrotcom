CFLAGS	:= -g -Wall -Wextra -O2
PROGRAM	:= parrotcom

all: $(PROGRAM)

$(PROGRAM): parrotcom.c
	$(CC) $(CFLAGS) -o $@ $^

install:
	install -D $(PROGRAM) $(DESTDIR)/usr/local/bin/$(PROGRAM)

distclean:
	rm -f $(PROGRAM)

clean:
	rm -f $(PROGRAM)
