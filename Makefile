CC     = clang++	
CPP    = clang++
CFLAGS = -g -MD -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual
CPPFLAGS = -g -MD -Wall -Wextra -Wshadow -Wpointer-arith -Wcast-qual -std=c++11 -stdlib=libstdc++
LFLAGS = -lm -stdlib=libstdc++
CFILES = 
CPPFILES = Vector2D.cpp Shapes.cpp GridState.cpp Robots.cpp main.cpp
OFILES = $(CFILES:.c=.o) $(CPPFILES:.cpp=.o)

all: main.x

main.x: $(OFILES)
	$(CC) $(OFILES) $(LFLAGS) -o main.x

%.o: %.c Makefile
	$(CC) $(CFLAGS) -o $@ -c $<

%.o: %.cpp Makefile
	$(CPP) $(CPPFLAGS) -o $@ -c $<
	
run: main.x
	./main.x 

grind: main.x
	valgrind --tool=memcheck --leak-check=full ./main.x

clean:
	rm -f $(OFILES)
	rm -f $(OFILES:.o=.d)
	rm -f main.x

-include: $(OFILES:.o=.d)

