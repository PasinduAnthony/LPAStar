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
	
	make_heap(U.begin(), U.end());

	calcKey(start);
	cout << start->key[0] << endl;

	U.push_back(start);
	push_heap(U.begin(),U.end());
}

void LpaStar::updateVertex(LpaStarCell* u) {
    if (u != start) {
		int mini = 100000;
        for (int pred = 0; pred < sizeof(u->predecessor); ++pred) {
            int c = 0;
			if((u->predecessor[pred]->x != 0) && (u->predecessor[pred]->y != 0)){
				c = SQRT_2;
			}else{
				c = 1;
			}

			if((u->predecessor[pred]->g + c)<mini){
				mini = u->predecessor[pred]->g + c;
			}
        }
		u->rhs = mini;
    }

    bool found = false;
	for (auto it = U.begin(); it != U.end(); ++it) {
		if (*it == u) {
			found = true;
			U.erase(it); // Remove u from the vector
			break; // Found u, no need to continue searching
		}
	}
    if (u->g != u->rhs) {
        // Update the cell's g value
        // u->g = u->rhs;

        // Add the cell back to the priority queue with the updated key
        // You'll need to implement this part based on your priority queue data structure
        // Calculate the key and add it to the queue
        calcKey(u);

        // u->open = true;
		U.push_back(u);
		push_heap(U.begin(),U.end());
    }
}

void LpaStar::computeShortestPath() {
    // Initialize a priority queue to manage the cells by their keys
    // You'll need to implement this priority queue based on your requirements
    // The queue should prioritize cells with the smallest key.

    // Add the goal cell to the priority queue with its key calculated from its g and h values
    // calcKey(goal);
    // goal->open = true;
    // Add the goal cell to the priority queue
	int hKey1 = U.back()->key[0];
	int hKey2 = U.back()->key[1];

	calcKey(goal);
	U.push_back(goal);
	int calKey1 = U.back()->key[0];
	int calKey2 = U.back()->key[1];

	cout << calKey2 << endl;
	cout << hKey1 <<endl;


	while(((hKey1 < calKey1) && (hKey2 < calKey2)) || (goal->rhs != goal->g)){
		LpaStarCell* u = U.back();
		U.pop_back();

		if(u->g > u->rhs){
			u->g = u->rhs;
			for(int dir = 0; dir < DIRECTIONS; ++dir){
				updateVertex(u->successor[dir]);
			}
		}else{
			u->g = INF;
			for(int dir = 0; dir < DIRECTIONS; ++dir){
				updateVertex(u->successor[dir]);
			}
			updateVertex(u);
		}
		break;
	}
    // while (/* stopping criteria not met */) {
    //     // Dequeue the cell with the smallest key from the priority queue
    //     LpaStarCell* currentCell = /* Dequeue the cell with the smallest key from the priority queue */;

    //     if (currentCell->key[0] >= currentCell->rhs) {
    //         if (/* stopping criteria met */) {
    //             break; // Terminate the search if the stopping criteria are met
    //         }

    //         currentCell->open = false;

    //         for (/* Each predecessor of 'currentCell' */) {
    //             // Compute the tentative cost to move from the predecessor to 'currentCell'
    //             double predecessorCost = /* Compute the cost based on your problem */;
    //             double tentativeG = currentCell->rhs + predecessorCost;

    //             // Check if the tentative g value is less than the predecessor's g value
    //             if (tentativeG < /* Predecessor's g value */) {
    //                 /* Update the predecessor's properties:
    //                    - Update the g value
    //                    - Update the rhs value
    //                    - Add the predecessor back to the priority queue with an updated key
    //                 */

    //                 // Calculate the key for the predecessor and add it to the priority queue
    //                 calcKey(/* Predecessor cell */);
    //                 /* Add the predecessor cell to the priority queue with the updated key */
    //                 /* Set the predecessor's 'open' flag to 'true' */
    //             }
    //         }
    //     } else {
    //         // The cell is inconsistent, update its key
    //         // Calculate the key for the current cell and add it back to the priority queue
    //         calcKey(currentCell);
    //         /* Add the current cell to the priority queue with the updated key */
    //         /* Set the current cell's 'open' flag to 'true' */
    //     }

    //     // Update statistics and handle other aspects of your algorithm
    // }
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




