

a.out: compileAll
	g++ *.o -o server -pthread
	rm -f *.o

compileAll: *.h *.cpp
	g++ -c *.cpp

run:
	./a.out
