#include <stdio.h>
#include <iostream>
#include <stdlib.h> /* calloc, exit, free */
#include <math.h> //sqrt, pow
#include <bits/stdc++.h>
#include<fstream>

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
	make_heap(PopOut.begin(), PopOut.end());
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
	// for (auto it = U.begin(); it != U.end(); ++it) {
	// 	if (*it == u) {
	// 		found = true;
	// 		break; // Found u, no need to continue searching
	// 	}
	// }
	
	// cout<< "Pop out size : " <<PopOut.size()<<endl;

	// bool found2 = false;
	// if(PopOut.size() > 0){
	// 	for (auto it = PopOut.begin(); it != PopOut.end(); ++it) {
	// 		if (*it == u) {
	// 			found2 = true;
	// 			break; // Found u, no need to continue searching
	// 		}
	// 	}
	// }

	// cout<< "Result : " << (found || found2) <<endl;

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
	
	// calc_H(s.x, s.y);
	cout << "updateVertex" <<endl;
    
	double mini = 100000;
	LpaStarCell* succ;
	LpaStarCell* miniSucc;
	// for (int pred = 0; pred < sizeof(u->successor); ++pred) {
	for (int i = 0; i < DIRECTIONS; i++) {
		if ((u->successor[i] != start) && (u->successor[i]->type == '0') && !(inHeap(u->successor[i]))) {	
		// if ((u->successor[i]->type == '0')) {			
			double c;
			succ = u->successor[i];
			succ->parent = u;
			c = getCost(succ, u);

			succ->h = calc_H(succ->x, succ->y);
			if((succ->g) == INF){
				// succ->g = c;
				succ->rhs = c;
			}else{
				// succ->g = succ->g + c;
				succ->rhs = succ->g + c;
			}
			

			// if(succ->rhs > u->g + c){
			// 	succ->parent = u;
            //     succ->rhs = u->g + c;

			// 	cout << "\n Rhs > G" << endl;
			// }

			// if(succ->h<mini){
			// 	mini = succ->h;
			// 	miniSucc = succ;
			// }

			// cout<<"\n Successor : "<< succ->x <<","<< succ->y << ", g = " <<succ->g << ", rhs = " << succ->rhs << ", h = " << succ->h;
			if (succ->g != succ->rhs) {
				calcKey(succ);
				U.push_back(succ);
				std::push_heap(U.begin(),U.end(), LpaStarCellComparator());
			}
		}
		// break;
	}
	std::sort_heap(U.begin(),U.end(), LpaStarCellComparator());
    // cout<<"\n Mini Successor : "<< miniSucc->x <<","<< miniSucc->y << ", g = " << miniSucc->g << ", rhs = " << miniSucc->rhs << ", h = " << miniSucc->h;

	
	bool found = false;
	// if (succ->g != succ->rhs) {

	// 	for (auto it = U.begin(); it != U.end(); ++it) {
	// 		if (*it == miniSucc) {
	// 			found = true;
	// 			U.erase(it);
	// 			cout << "Erased" <<endl;
	// 			break; // Found u, no need to continue searching
	// 		}
	// 	}

	// 	calcKey(miniSucc);
	// 	U.push_back(miniSucc);
	// 	std::push_heap(U.begin(),U.end());
	// }

    found = false;

	fstream f;
	ofstream fout;
	ifstream fin;
	fin.open("a2.txt");
	fout.open ("a2.txt",ios::app);
	if(fin.is_open()){
		fout<<"\n Iteration 1";

		for (auto it = U.begin(); it != U.end(); ++it) {
			miniSucc = *it;
			
			fout<<"\n"<< miniSucc->x <<","<< miniSucc->y << ", g = " <<miniSucc->g << ", rhs = " << miniSucc->rhs << ", h = " << miniSucc->h;
					

				// cout << "Path X : " << u->x << ", Y : " << u->y <<endl;
			if (*it == u) {
				found = true;

				PopOut.push_back(miniSucc);
				std::push_heap(PopOut.begin(),PopOut.end());

				fout<<"\n Pop : " << miniSucc->x <<","<< miniSucc->y << ", g = " <<miniSucc->g << ", rhs = " << miniSucc->rhs << ", h = " << miniSucc->h;
				cout <<"\n Pop : " << miniSucc->x <<","<< miniSucc->y << ", g = " <<miniSucc->g << ", rhs = " << miniSucc->rhs << ", h = " << miniSucc->h;
				
				U.erase(it); // Remove u from the vector
				break; // Found u, no need to continue searching
			}
		}
		fout<<"\n Goal : "<<goal->x<<","<<goal->y;
		fout<<"\n End of iteration 1";
		fout<<"\n...............................................";
		cout <<"\n"<< u->x <<","<< u->y <<endl;
		fin.close();
		fout.close();
		string word;
		f.open("a2.txt");
	}
	// cout << "Heap Size : " << U.size() <<endl;

	cout << "\n --------------------------------------------------------------------" <<endl;
	// std::cout << "Elements in the heap: ";
    for (LpaStarCell* num : U) {
		
        cout << "Path X : " << num->x << ", Y : " << num->y << ", G : " << num->g << ", RHS : " << num->rhs << ", Key : " << num->key[0] <<","<<num->key[1] <<endl;  
		
    }
    
	cout << "\n --------------------------------------------------------------------" <<endl;

	// cout << U.front()->x <<","<< U.front()->y <<endl;

	
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

	LpaStarCell* u = U.front();
	
	// cout << calc_H(U.back()->x, U.back()->y) <<endl;
	updateHValues();
	
	calcKey(goal);
	int calKey1 = goal->key[0];
	int calKey2 = goal->key[1];
	// cout << ((hKey1 < calKey1) && (hKey2 < calKey2)) <<endl;
	
	int count = 0;
	
	while(!U.empty() && (count < 17) && ((u->key[0] < calKey1) && (u->key[1] < calKey2)) || (goal->rhs != goal->g) ){
		cout << "inside while" <<endl;
		
		++count; 
		u = U.front();
		cout<<"\n Top value : "<< u->x <<","<< u->y << ", g = " <<u->g << ", rhs = " << u->rhs << ", h = " << u->h;
		
		if (u->x == goal->x && u->y == goal->y) {
            break;  // Goal reached
        }

		// U.pop_back();
		if(u->g > u->rhs){
			u->g = u->rhs;
			// for(int dir = 0; dir < DIRECTIONS; ++dir){
			// 	updateVertex(u->successor[dir]);
			// }
			cout << "inside if " <<endl;
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
			cout << "inside else" <<endl;
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
		// updateVertex(u);

		
		updateHValues();
	
		calcKey(goal);
	
		calKey1 = goal->key[0];
		calKey2 = goal->key[1];
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