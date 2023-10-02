#include <stdio.h>
#include <iostream>
#include <stdlib.h> /* calloc, exit, free */
#include <math.h> //sqrt, pow
#include <bits/stdc++.h>

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
	make_heap(U.begin(), U.end());
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].g = INF;
		   maze[i][j].rhs = INF;
		}
	}
	
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
	


	//for debugging only
	//~ for(int i=0; i < rows; i++){
	   //~ for(int j=0; j < cols; j++){
		   //~ //cout << maze[i][j].g << ", ";
			//~ cout << maze[i][j].rhs << ", ";
			
		//~ }
		//~ cout << endl;
	//~ }
	
	
	updateHValues();
	calcKey(start);
	// cout << start->key[0] << endl;
	// cout << start->key[1] << endl;

	U.push_back(start);
	push_heap(U.begin(),U.end());
}

void LpaStar::updateVertex(LpaStarCell* u) {
	
	// calc_H(s.x, s.y);
	cout << "updateVertex" <<endl;
    
	double mini = 100000;
	LpaStarCell* succ;
	LpaStarCell* miniSucc;
	// for (int pred = 0; pred < sizeof(u->successor); ++pred) {
	for (int i = 0; i < DIRECTIONS; i++) {
		if ((u->successor[i] != start) && (u->successor[i]->type == '0')) {	
		// if ((u->successor[i]->type == '0')) {			
			double c = 0;
			succ = u->successor[i];
			if(((succ->x == (u->x)-1) && (succ->y == (u->y)-1)) || ((succ->x == (u->x)+1) && (succ->y == (u->y)-1)) || ((succ->x == (u->x)-1) && (succ->y == (u->y)+1)) || ((succ->x == (u->x)+1) && (succ->y == (u->y)+1))){
				c = SQRT_2;
			}else{
				c = 1;
			}

			u->successor[i]->h = calc_H(u->successor[i]->x, u->successor[i]->y);
			if((u->successor[i]->g) == INF){
				u->successor[i]->rhs = c;
			}else{
				u->successor[i]->g = u->successor[i]->g + c;
				u->successor[i]->rhs = u->successor[i]->g + c;
			}
			

			if(u->successor[i]->h<mini){
				mini = succ->h;
				miniSucc = succ;
				cout << mini <<endl;
				cout << "Current X : " << succ->x << ", Y : " << succ->y <<endl; 
				cout << "rhs" <<endl;
				cout << u->successor[i]->rhs <<endl;
			}
			cout << "Start X : " << start->x << ", Y : " << start->y <<endl; 
			cout << "Traversable X : " << succ->x << ", Y : " << succ->y <<endl; 
			cout << "Mini rhs : "<< mini <<endl; 
		}
		// cout << "DIRECTIONS X : " << u->successor[i]->x << ", Y : " << u->successor[i]->y <<endl;  
	}

	cout << "hello" <<endl;
	// u->rhs = succ->rhs;

	cout << "Mini rhs 2 : "<< miniSucc->h <<endl;

		// int c = 0;
		// if((u->x != 0) && (u->y != 0)){
		// 	c = SQRT_2;
		// }else{
		// 	c = 1;
		// }

		// if((u->g + c)<mini){
		// 	mini = u->g + c;
		// }
		
    

    bool found = false;
	for (auto it = U.begin(); it != U.end(); ++it) {
		if (*it == u) {
			found = true;
			U.erase(it); // Remove u from the vector
			break; // Found u, no need to continue searching
		}
	}
	cout << "U removed : " << found <<endl;

    if (u->g != u->rhs) {
        // Update the cell's g value
        // u->g = u->rhs;

        // Add the cell back to the priority queue with the updated key
        // You'll need to implement this part based on your priority queue data structure
        // Calculate the key and add it to the queue
       
	    // calcKey(u);
		// cout << "g not equal to rhs" <<endl;
        // // u->open = true;
		// calc_H(u->x, u->y);
		// U.push_back(u);
		// push_heap(U.begin(),U.end());
    }
}

void LpaStar::computeShortestPath() {
    // Initialize a priority queue to manage the cells by their keys
    // You'll need to implement this priority queue based on your requirements
    // The queue should prioritize cells with the smallest key.

    // Add the goal cell to the priority queue with its key calculated from its g and h values
    // calcKey(goal);
   
    // Add the goal cell to the priority queue
	int hKey1 = INF;
	int hKey2 = INF;

	if (!U.empty()) {
        hKey1 = U.back()->key[0];
		hKey2 = U.back()->key[1];
    }
	
	// cout << calc_H(U.back()->x, U.back()->y) <<endl;
	updateHValues();
	// cout << U.back()->type <<endl;
	// cout << U.back()->x <<endl;
	// cout << U.back()->y <<endl;

	// cout << rows <<endl;
	// cout << cols <<endl;
	// cout << maze[(U.back()->y)-1][U.back()->x].type <<endl;
	// cout << maze[U.back()->y][(U.back()->x)-1].type <<endl;
	
	// cout << goal->x <<endl;
	// cout << goal->y <<endl;
	// cout << goal->h <<endl;
	


	calcKey(goal);
	// cout << goal->key[0] <<endl;
	// cout << goal->key[1] <<endl;
	// U.push_back(goal);
	int calKey1 = goal->key[0];
	int calKey2 = goal->key[1];
	// cout << ((hKey1 < calKey1) && (hKey2 < calKey2)) <<endl;
	// cout << U.back()->g <<endl;
	
	// int i = 0;
	// while (i < 5) {
	// 	cout << "while loop" << "\n";
	// 	i++;
	// }
	
	
	// while(((hKey1 < calKey1) && (hKey2 < calKey2)) || (goal->rhs != goal->g)){
	// 	cout << "inside while" <<endl;
	// 	LpaStarCell* u = U.back();
	// 	U.pop_back();
	// 	cout << "after popping" <<endl;
	// 	// // cout << U.back()->x <<endl;
	// 	// cout << cols <<endl;
	// 	if(u->g > u->rhs){
	// 		u->g = u->rhs;
	// 		LpaStarCell* s = u;
	// 		if(((u->x)-1 < cols) && ((u->y)-1 < rows) && ((u->x)-1 >= 0) && ((u->y)-1 >= 0)){
	// 			s = &maze[(u->y)-1][(u->x)-1];
	// 			if(s->type = 0){
	// 				updateVertex(s);
	// 			}
	// 		}
	// 	}
	// 	cout << "if" <<endl;
	// 	break;
	// }

	while(!U.empty() && ((hKey1 < calKey1) && (hKey2 < calKey2)) || (goal->rhs != goal->g)){
		cout << "inside while" <<endl;
		// hKey1 = U.back()->key[0];
		// hKey2 = U.back()->key[1];

		// calcKey(goal);
		// calKey1 = goal->key[0];
		// calKey2 = goal->key[1];

		LpaStarCell* u = U.back();
		// U.pop_back();
		cout << "after popping" <<endl;

		if(u->g > u->rhs){
			u->g = u->rhs;
			// for(int dir = 0; dir < DIRECTIONS; ++dir){
			// 	updateVertex(u->successor[dir]);
			// }
			LpaStarCell* s = u;
			if(((u->x)-1 < cols) && ((u->y)-1 < rows) && ((u->x)-1 >= 0) && ((u->y)-1 >= 0)){
				s = &maze[(u->y)-1][(u->x)-1];
				u->successor[0] = s;
			}
			if(((u->x) < cols) && ((u->y)-1 < rows) && ((u->x) >= 0) && ((u->y)-1 >= 0)){
				s = &maze[(u->y)-1][(u->x)];	
				u->successor[1] = s;
			}
			if(((u->x)+1 < cols) && ((u->y)-1 < rows) && ((u->x)+1 >= 0) && ((u->y)-1 >= 0)){
				s = &maze[(u->y)-1][(u->x)+1];
				u->successor[2] = s;
			}
			if(((u->x)-1 < cols) && ((u->y) < rows) && ((u->x)-1 >= 0) && ((u->y) >= 0)){
				s = &maze[(u->y)][(u->x)-1];
				u->successor[3] = s;
			}
			if(((u->x)+1 < cols) && ((u->y) < rows) && ((u->x)+1 >= 0) && ((u->y) >= 0)){
				s = &maze[(u->y)][(u->x)+1];
				u->successor[4] = s;
			}
			if(((u->x)-1 < cols) && ((u->y)+1 < rows) && ((u->x)-1 >= 0) && ((u->y)+1 >= 0)){
				s = &maze[(u->y)+1][(u->x)-1];
				u->successor[5] = s;
			}
			if(((u->x) < cols) && ((u->y)+1 < rows) && ((u->x) >= 0) && ((u->y)+1 >= 0)){
				s = &maze[(u->y)+1][(u->x)];
				u->successor[6] = s;
			}
			if(((u->x)+1 < cols) && ((u->y)+1 < rows) && ((u->x)+1 >= 0) && ((u->y)+1 >= 0)){
				s = &maze[(u->y)+1][(u->x)+1];
				u->successor[7] = s;
			}
			// updateVertex(u);
		}else{
			u->g = INF;
			// for(int dir = 0; dir < DIRECTIONS; ++dir){
			// 	updateVertex(u->successor[dir]);
			// }
			// updateVertex(u);
			LpaStarCell* s = u;
			if(((u->x)-1 < cols) && ((u->y)-1 < rows) && ((u->x)-1 >= 0) && ((u->y)-1 >= 0)){
				s = &maze[(u->y)-1][(u->x)-1];
				u->successor[0] = s;
			}
			if(((u->x) < cols) && ((u->y)-1 < rows) && ((u->x) >= 0) && ((u->y)-1 >= 0)){
				s = &maze[(u->y)-1][(u->x)];	
				u->successor[1] = s;
			}
			if(((u->x)+1 < cols) && ((u->y)-1 < rows) && ((u->x)+1 >= 0) && ((u->y)-1 >= 0)){
				s = &maze[(u->y)-1][(u->x)+1];
				u->successor[2] = s;
			}
			if(((u->x)-1 < cols) && ((u->y) < rows) && ((u->x)-1 >= 0) && ((u->y) >= 0)){
				s = &maze[(u->y)][(u->x)-1];
				u->successor[3] = s;
			}
			if(((u->x)+1 < cols) && ((u->y) < rows) && ((u->x)+1 >= 0) && ((u->y) >= 0)){
				s = &maze[(u->y)][(u->x)+1];
				u->successor[4] = s;
			}
			if(((u->x)-1 < cols) && ((u->y)+1 < rows) && ((u->x)-1 >= 0) && ((u->y)+1 >= 0)){
				s = &maze[(u->y)+1][(u->x)-1];
				u->successor[5] = s;
			}
			if(((u->x) < cols) && ((u->y)+1 < rows) && ((u->x) >= 0) && ((u->y)+1 >= 0)){
				s = &maze[(u->y)+1][(u->x)];
				u->successor[6] = s;
			}
			if(((u->x)+1 < cols) && ((u->y)+1 < rows) && ((u->x)+1 >= 0) && ((u->y)+1 >= 0)){
				s = &maze[(u->y)+1][(u->x)+1];
				u->successor[7] = s;
			}
			// updateVertex(u);
		}
		updateVertex(u);

		if (!U.empty()) {
			hKey1 = U.back()->key[0];
			hKey2 = U.back()->key[1];
		}

		updateHValues();
	
		calcKey(goal);
	
		int calKey1 = goal->key[0];
		int calKey2 = goal->key[1];
		// break;
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
	
	int diffY = abs(goal->y - y);
	int diffX = abs(goal->x - x);
	
	//maze[y][x].h = (double)maxValue(diffY, diffX);
	return (double)maxValue(diffY, diffX);
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




