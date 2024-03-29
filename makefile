search.exe		: 	main.o  transform.o AstarSearch.o LPAstar.o gridworld.o graphics.o 
	g++ -Wl,-s -o search.exe main.o transform.o AstarSearch.o LPAstar.o gridworld.o graphics.o -l lpa_star->computeShortestPath();gdi32 
			
main.o		:	main.cpp graphics.h transform.h AstarSearch.h LPAstar.h gridworld.h globalvariables.h
	g++ -c -std=c++11 -O2  -fpermissive -fconserve-space -Wno-write-strings  main.cpp
	
transform.o		:	 transform.cpp transform.h
	g++ -c -std=c++11 -O2  -fpermissive -fconserve-space -Wno-write-strings  transform.cpp	
	
AstarSearch.o	:	 AstarSearch.cpp AstarSearch.h
	g++ -c -std=c++11 -O2  -fpermissive -fconserve-space -Wno-write-strings  AstarSearch.cpp

LPAstar.o	:	 LPAstar.cpp LPAstar.h
	g++ -c -std=c++11 -O2  -fpermissive -fconserve-space -Wno-write-strings  LPAstar.cpp

gridworld.o	:	 gridworld.cpp gridworld.h
	g++ -c -std=c++11 -O2  -fpermissive -fconserve-space -Wno-write-strings  gridworld.cpp

graphics.o		:	 graphics.cpp graphics.h
	g++ -c -std=c++11 -O2  -fpermissive -fconserve-space -Wno-write-strings  graphics.cpp
	
clean:
	del *.o
	del *.exe
