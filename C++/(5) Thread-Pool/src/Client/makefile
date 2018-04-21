

a.out: compileAll
	g++ *.o -o client
	rm -f *.o

compileAll: *.h *.cpp
	g++ -c *.cpp

run:
	./a.out
