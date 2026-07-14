objs = startup_bulk.o

CFLAGS = -Wall -O2 -c
LDFLAGS = -laio -flto

LDFLAGS += '-laio'

all: startup_bulk

startup_bulk: $(objs)
	gcc $(objs) -o startup_bulk $(LDFLAGS)

%.o : %.c
	gcc $(CFLAGS) -c $< -o $@

clean:
	rm -rf startup_bulk *.o
