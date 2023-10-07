#include <stdio.h>
#include <iostream>
#include <stdlib.h> /* calloc, exit, free */
#include <math.h> //sqrt, pow
#include <bits/stdc++.h>
#include<fstream>
#include <chrono>

#include "LPAstar.h"
#include "gridworld.h"

using namespace std;


 LpaStar::LpaStar(int rows_, int cols_, unsigned int _heuristic, string _gridWorldName){
	     rows = rows_;
	     cols = cols_;

	     heuristic = _heuristic;
	     if(heuristic == CHEBYSHEV){
	     	strHeuristic = "CHEBYSHEV";

	     	if(DEBUG){
	     		cout << "\nh= CHEBYSHEV" << endl;
	     	}
	     	
	     } else if (heuristic == EUCLIDEAN){
	     	strHeuristic = "EUCLIDEAN";
	     	if(DEBUG){
	     		cout << "\nh= EUCLIDEAN" << endl;
	     	}
	     	
	     }

	     gridWorldName = _gridWorldName;
	 
		 //Allocate memory 
		 maze.resize(rows);
		 for(int i=0; i < rows; i++){
		   maze[i].resize(cols);
		 }
}

void LpaStar::initialise(int startX, int startY, int goalX, int goalY){
	make_heap(U.begin(), U.end(), LpaStarCellComparator());
	make_heap(PopOut.begin(), PopOut.end(),LpaStarCellComparatorShortestPath());
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].g = INF;
		   maze[i][j].rhs = INF;
		}
	}


	tempStartY = startY;
	tempStartX = startX;

	// start = new LpaStarCell;
	// goal = new LpaStarCell;

	start = &maze[startY][startX];
	goal = &maze[goalY][goalX];
	
	//START VERTEX
	start->g = INF;
	start->rhs = 0.0;
	start->x = startX;
	start->y = startY;
	
	//GOAL VERTEX
	goal->g = INF;
	goal->rhs = INF;
	goal->x = goalX;
	goal->y = goalY;
	//---------------------
	maze[start->y][start->x].g = start->g;
	maze[start->y][start->x].rhs = start->rhs;
	
	maze[goal->y][goal->x].g = goal->g;
	maze[goal->y][goal->x].rhs = goal->rhs;
	//---------------------

	stateExpansions=0;    
    maxQLength=0;
    vertexAccesses=0;
    pathLength=0.0;
    runningTime=0.0;
	
	updateHValues();
	calcKey(start);
	U.push_back(start);
	push_heap(U.begin(),U.end());
}

double LpaStar::getCost(LpaStarCell* succ, LpaStarCell* u){
	double c;
	if(((succ->x == (u->x)-1) && (succ->y == (u->y)-1)) || ((succ->x == (u->x)+1) && (succ->y == (u->y)-1)) || ((succ->x == (u->x)-1) && (succ->y == (u->y)+1)) || ((succ->x == (u->x)+1) && (succ->y == (u->y)+1))){
		c = SQRT_2;
	}else{
		c = 1;
	}

	return c;
}


bool LpaStar::inHeap(LpaStarCell* u){
	bool found = false;
	for (const LpaStarCell* cell : U) {
        // Access and print the relevant fields of the LpaStarCell object
		if((cell->x == u->x) && (cell->y == u->y)){
			found = true;
			break;
		}
        // Add more fields as needed
    }

	for (const LpaStarCell* cell : PopOut) {
        // Access and print the relevant fields of the LpaStarCell object
		if((cell->x == u->x) && (cell->y == u->y)){
			found = true;
			break;
		}
        // Add more fields as needed
    }
	return (found);
}

void LpaStar::updateVertex(LpaStarCell* u) {
	double mini = 100000;
	LpaStarCell* succ;
	LpaStarCell* miniSucc;
	
	for (int i = 0; i < DIRECTIONS; i++) {
		if ((u->successor[i] != start) && ((u->successor[i]->type == '0') || (u->successor[i]->type == '7') || u->successor[i]->type == '8' || u->successor[i]->type == '9') && !(inHeap(u->successor[i]))) {	
		// if ((u->successor[i]->type == '0')) {			
			double c;
			succ = u->successor[i];
			succ->parent = u;
			c = getCost(succ, u);
			++vertextAccess;
			succ->h = calc_H(succ->x, succ->y);
			
			if(succ->rhs > u->g + c){
				succ->parent = u;
                succ->rhs = u->g + c;
			}else{
				succ->rhs = c;
			}
			

			
			if (succ->g != succ->rhs) {
				calcKey(succ);
				U.push_back(succ);

				maze[succ->y][succ->x].g = succ->g;
				maze[succ->y][succ->x].rhs = succ->rhs; 
				maze[succ->y][succ->x].h = succ->h;
				maze[succ->y][succ->x].key[0] = succ->key[0];
				maze[succ->y][succ->x].key[1] = succ->key[1];
				
				std::push_heap(U.begin(),U.end(), LpaStarCellComparator());
			}
			
			// if((u->successor[i]->type == '8')){
			// 	u->successor[i]->type = '0';
			// 	maze[u->successor[i]->y][u->successor[i]->x].type = u->successor[i]->type;
			// }else if((u->successor[i]->type == '9')){
			// 	u->successor[i]->type = '1';
			// 	maze[u->successor[i]->y][u->successor[i]->x].type = u->successor[i]->type;
			// }
		}
	}
	std::sort_heap(U.begin(),U.end(), LpaStarCellComparator());

	bool found = false;
	
    fstream f;
	ofstream fout;
	ifstream fin;
	fin.open("a2.txt");
	fout.open ("a2.txt",ios::app);
	if(fin.is_open()){
		for (auto it = U.begin(); it != U.end(); ++it) {
			miniSucc = *it;
			if (*it == u) {
				found = true;

				PopOut.push_back(miniSucc);
				push_heap(PopOut.begin(),PopOut.end(),LpaStarCellComparatorShortestPath());

				// maze[miniSucc->y][miniSucc->x].g = miniSucc->g;
				// maze[miniSucc->y][miniSucc->x].rhs = miniSucc->rhs; 
				// maze[miniSucc->y][miniSucc->x].h = miniSucc->h;
				// maze[miniSucc->y][miniSucc->x].key[0] = miniSucc->key[0];
				// maze[miniSucc->y][miniSucc->x].key[1] = miniSucc->key[1];

				fout<<"\n Pop : " << miniSucc->x <<","<< miniSucc->y << ", g = " <<miniSucc->g << ", rhs = " << miniSucc->rhs << ", h = " << miniSucc->h <<" key :"<<miniSucc->key[0] <<","<<miniSucc->key[1];
				U.erase(it); // Remove u from the vector
				break; // Found u, no need to continue searching
			}
		}
		fin.close();
		fout.close();
		string word;
		f.open("a2.txt");
	}
	if(maxQLenght < U.size()){
		maxQLenght = U.size();
	}
}

void LpaStar::updateNeighbours(int x, int y, char type){
	bool found = false;
	if((maze[y][x].g != INF) && (maze[y][x].rhs != INF)){
		for (LpaStarCell* cell : U) {
			cout << cell->x <<"," <<cell->y << " g : " <<cell->g << " rhs : " << cell->rhs<< "Key : " << cell->key[0] <<","<<cell->key[1]<<endl;
			// Access and print the relevant fields of the LpaStarCell object
			if((cell->x == x) && (cell->y == y)){				
				cout<< "Found in priority queue" <<endl;
				temp = cell;
				found = true;
				// break;
			}
			// Add more fields as needed
		}
		cout << "---------------------=--------------" <<endl;
		if(!found){
			for (LpaStarCell* cell : PopOut) {
				cout << cell->x <<"," <<cell->y << " g : " <<cell->g << " rhs : " << cell->rhs<< "Key : " << cell->key[0] <<","<<cell->key[1]<<endl;
				// Access and print the relevant fields of the LpaStarCell object
				if((cell->x == x) && (cell->y == y)){				
					cout<< "Found in pop queue" <<endl;
					temp = cell;
					found = true;
					// break;
				}
				// Add more fields as needed
			}
		}
		cout << "---------------------=--------------" <<endl;
		if(!found){
			temp = &maze[y][x];
		}
		cout<< temp->x <<","<<temp->y <<endl;
		temp->type = '1';

		if(((x-1 < cols) && (y-1 < rows)) && ((x-1 >= 0) && (y-1 >= 0))){
			temp->successor[0] = &maze[y-1][x-1];
		}
		if(((x < cols) && (y-1 < rows)) && ((x >= 0) && (y-1 >= 0))){
			temp->successor[1] = &maze[y-1][x];
		}
		if(((x+1 < cols) && (y-1 < rows)) && ((x+1 >= 0) && (y-1 >= 0))){
			temp->successor[2] = &maze[y-1][x+1];
		}
		if(((x-1 < cols) && (y < rows)) && ((x-1 >= 0) && (y >= 0))){
			temp->successor[3] = &maze[y][x-1];
		}
		if(((x+1 < cols) && (y < rows)) && ((x+1 >= 0) && (y >= 0))){
			temp->successor[4] = &maze[y][x+1];
		}
		if(((x-1 < cols) && (y+1 < rows)) && ((x-1 >= 0) && (y+1 >= 0))){
			temp->successor[5] = &maze[y+1][x-1];
		}
		if(((x < cols) && (y+1 < rows)) && ((x >= 0) && (y+1 >= 0))){
			temp->successor[6] = &maze[y+1][x];
		}
		if(((x+1 < cols) && (y+1 < rows)) && ((x+1 >= 0) && (y+1 >= 0))){
			temp->successor[7] = &maze[y+1][x+1];
		}
		

		for (int i = 0; i < DIRECTIONS; i++) {
			// updateVertex(temp->successor[i]);
			temp->successor[i]->g = INF;
			temp->successor[i]->rhs = INF;
			temp->successor[i]->key[0] = 0;
			temp->successor[i]->key[1] = 0;
		}

		

		temp->g = INF;
		temp->rhs = INF;
		temp->key[0] = 0;
		temp->key[1] = 0;

		
		temp = &maze[tempStartY][tempStartX];

		U.push_back(temp);

		std::push_heap(U.begin(),U.end(),LpaStarCellComparator());
		std::sort_heap(U.begin(),U.end(), LpaStarCellComparator());

		cout << U.front()->x <<"," <<U.front()->y << " g : " <<U.front()->g << " rhs : " << U.front()->rhs<< "Key : " << U.front()->key[0] <<","<<U.front()->key[1]<<endl;
		cout << "\n" <<endl;
		cout << "\n "<<endl;
		for (LpaStarCell* cell : U) {
			cout << cell->x <<"," <<cell->y << " g : " <<cell->g << " rhs : " << cell->rhs<< "Key : " << cell->key[0] <<","<<cell->key[1]<<endl;
			// Access and print the relevant fields of the LpaStarCell object
			if((cell->x == x) && (cell->y == y)){				
				cout<< "Found in priority queue" <<endl;
			}
			// Add more fields as needed
		}
		cout << "PopOut "<<endl;
		
		for (LpaStarCell* cell : PopOut) {
			PopOut.pop_back();
		}

		for (LpaStarCell* cell : PopOut) {
			cout << cell->x <<"," <<cell->y << " g : " <<cell->g << " rhs : " << cell->rhs<< "Key : " << cell->key[0] <<","<<cell->key[1]<<endl;
			// Access and print the relevant fields of the LpaStarCell object
			if((cell->x == x) && (cell->y == y)){				
				cout<< "Found in pop queue" <<endl;
			}
			// Add more fields as needed
		}
		
		// computeShortestPath();	
	}	
	
	
}

void LpaStar::computeShortestPath() {
    // Add the goal cell to the priority queue
	int hKey1 = INF;
	int hKey2 = INF;

	// Start the clock
    auto startTime = std::chrono::high_resolution_clock::now();

	LpaStarCell* u = U.front();;
	
	updateHValues();
	
	calcKey(goal);
	int calKey1 = goal->key[0];
	int calKey2 = goal->key[1];
		
	int count = 0;
	
	while(!U.empty() && ((u->key[0] < calKey1) && (u->key[1] < calKey2)) || (goal->rhs != goal->g) ){
		
		u = U.front();
		
		if(u->g > u->rhs){
			u->g = u->rhs;
			if((((u->x)-1 < cols) && ((u->y)-1 < rows)) && (((u->x)-1 >= 0) && ((u->y)-1 >= 0))){
				u->successor[0] = &maze[(u->y)-1][(u->x)-1];
			}
			if((((u->x) < cols) && ((u->y)-1 < rows)) && (((u->x) >= 0) && ((u->y)-1 >= 0))){
				u->successor[1] = &maze[(u->y)-1][(u->x)];
			}
			if((((u->x)+1 < cols) && ((u->y)-1 < rows)) && (((u->x)+1 >= 0) && ((u->y)-1 >= 0))){
				u->successor[2] = &maze[(u->y)-1][(u->x)+1];
			}
			if((((u->x)-1 < cols) && ((u->y) < rows)) && (((u->x)-1 >= 0) && ((u->y) >= 0))){
				u->successor[3] = &maze[(u->y)][(u->x)-1];
			}
			if((((u->x)+1 < cols) && ((u->y) < rows)) && (((u->x)+1 >= 0) && ((u->y) >= 0))){
				u->successor[4] = &maze[(u->y)][(u->x)+1];
			}
			if((((u->x)-1 < cols) && ((u->y)+1 < rows)) && (((u->x)-1 >= 0) && ((u->y)+1 >= 0))){
				u->successor[5] = &maze[(u->y)+1][(u->x)-1];
			}
			if((((u->x) < cols) && ((u->y)+1 < rows)) && (((u->x) >= 0) && ((u->y)+1 >= 0))){
				u->successor[6] = &maze[(u->y)+1][(u->x)];
			}
			if((((u->x)+1 < cols) && ((u->y)+1 < rows)) && (((u->x)+1 >= 0) && ((u->y)+1 >= 0))){
				u->successor[7] = &maze[(u->y)+1][(u->x)+1];
			}
			updateVertex(u);
		}else{
			u->g = INF;
			if((((u->x)-1 < cols) && ((u->y)-1 < rows)) && (((u->x)-1 >= 0) && ((u->y)-1 >= 0)) && (((u->x)-1 != u->x) && ((u->y)-1 != u->y))){
				u->successor[0] = &maze[(u->y)-1][(u->x)-1];
			}
			if((((u->x) < cols) && ((u->y)-1 < rows)) && (((u->x) >= 0) && ((u->y)-1 >= 0)) && (((u->x) != u->x) && ((u->y)-1 != u->y))){
				u->successor[1] = &maze[(u->y)-1][(u->x)];
			}
			if((((u->x)+1 < cols) && ((u->y)-1 < rows)) && (((u->x)+1 >= 0) && ((u->y)-1 >= 0)) && (((u->x)+1 != u->x) && ((u->y)-1 != u->y))){
				u->successor[2] = &maze[(u->y)-1][(u->x)+1];
			}
			if((((u->x)-1 < cols) && ((u->y) < rows)) && (((u->x)-1 >= 0) && ((u->y) >= 0)) && (((u->x)-1 != u->x) && ((u->y) != u->y))){
				u->successor[3] = &maze[(u->y)][(u->x)-1];
			}
			if((((u->x)+1 < cols) && ((u->y) < rows)) && (((u->x)+1 >= 0) && ((u->y) >= 0)) && (((u->x)-1 != u->x) && ((u->y) != u->y))){
				u->successor[4] = &maze[(u->y)][(u->x)+1];
			}
			if((((u->x)-1 < cols) && ((u->y)+1 < rows)) && (((u->x)-1 >= 0) && ((u->y)+1 >= 0)) && (((u->x)-1 != u->x) && ((u->y)+1 != u->y))){
				u->successor[5] = &maze[(u->y)+1][(u->x)-1];
			}
			if((((u->x) < cols) && ((u->y)+1 < rows)) && (((u->x) >= 0) && ((u->y)+1 >= 0)) && (((u->x) != u->x) && ((u->y)+1 != u->y))){
				u->successor[6] = &maze[(u->y)+1][(u->x)];
			}
			if((((u->x)+1 < cols) && ((u->y)+1 < rows)) && (((u->x)+1 >= 0) && ((u->y)+1 >= 0)) && (((u->x)+1 != u->x) && ((u->y)+1 != u->y))){
				u->successor[7] = &maze[(u->y)+1][(u->x)+1];
			}
			updateVertex(u);
		}
		++count; 
		
		updateHValues();
	
		calcKey(goal);
	
		calKey1 = goal->key[0];
		calKey2 = goal->key[1];
	}

	cout << "\n-----------Goal Reached-----------" <<endl;
	cout << "\nState Expansions : " << count <<endl;
	cout << "Max_Q_Length : " << maxQLenght <<endl;
	cout << "Vertext_Accessed : "<< vertextAccess <<endl;
	cout << "Path Lenght : "<<goal->rhs<<endl;

	// Stop the clock
    auto stop = std::chrono::high_resolution_clock::now();

	// Calculate the duration
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);

    std::cout << "Runtime: " << duration.count() << " milliseconds" << std::endl;
	
	std::sort_heap(PopOut.begin(),PopOut.end(), LpaStarCellComparatorShortestPath());

	fstream f;
	ofstream fout;
	ifstream fin;
	fin.open("a2.txt");
	fout.open ("a2.txt",ios::app);
	fout<<"\n";
	int count2 = 0;
	fout<< "\n Shortest Path" ;
	double tempRhs = PopOut.front()->rhs;				
	if(fin.is_open()){
		for (LpaStarCell* miniSucc : PopOut) {	
			
			if((tempRhs > miniSucc->rhs) && (tempRhs != miniSucc->rhs)){
				fout<<"\n" << miniSucc->x <<","<< miniSucc->y << ", g = " <<miniSucc->g << ", rhs = " << miniSucc->rhs << ", h = " << miniSucc->h <<" key :"<<miniSucc->key[0] <<","<<miniSucc->key[1];				
				tempRhs = miniSucc->rhs;
			}
			++count2;
		}
		
		fin.close();
		fout.close();
		string word;
		f.open("a2.txt");
	}
	
}

void LpaStar::initialPlanning(){
        episode=INITIAL_PLANNING;
        strEpisode = "initial_planning";
}

void LpaStar::finalPlanning(){
        episode = FINAL_PLANNING;
        strEpisode = "final_planning";

}

double LpaStar::minValue(double g_, double rhs_){
	if(g_ <= rhs_){
		return g_;
	} else {
		return rhs_;
	}	
}

int LpaStar::maxValue(int v1, int v2){
	
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

double LpaStar::calc_H(int x, int y){
	double val;
	int diffY;
	int diffX;
	if(strHeuristic == "CHEBYSHEV"){
		diffY = abs(goal->y - y);
		diffX = abs(goal->x - x);
		val = maxValue(diffY, diffX);
	}else if(strHeuristic == "EUCLIDEAN"){
		diffY = (goal->y -y)^2;
		diffX = (goal->x -x)^2;
		val = sqrt(maxValue(diffY, diffX));
	}
	return val;
}

void LpaStar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].h = calc_H(j, i);
		}
	}
	
	start->h = calc_H(start->x, start->y);
	goal->h = calc_H(goal->x, goal->y);
}

void LpaStar::calcKey(int x, int y){
	double key1, key2;
	
	key2 = minValue(maze[y][x].g, maze[y][x].rhs);
	key1 = key2 + maze[y][x].h;
}


void LpaStar::calcKey(LpaStarCell *cell){
	double key1, key2;
	
	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + cell->h;
	
	cell->key[0] = key1;
	cell->key[1] = key2;
}

void LpaStar::updateAllKeyValues(){	
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   calcKey(&maze[i][j]);
		}
	}
	
	calcKey(start);
	calcKey(goal);
}