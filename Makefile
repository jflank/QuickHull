test: QuickHull.o Point2D.o
	g++ -g -o test QuickHull.o Point2D.o

QuickHull.o: QuickHull.cpp
	g++ -std=c++11 -g -c QuickHull.cpp

Point2D.o: Point2D.cpp
	g++ -std=c++11 -g -c Point2D.cpp

run:
	./test 
