#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector> 
#include <iostream> 
#include <iomanip> 
#include "globalVariables.h"
#include <queue>

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

#define INITIAL_PLANNING 1
#define FINAL_PLANNING 2

class LpaStar{

public:
    LpaStar(int rows, int cols, unsigned int _heuristic, string _gridWorldName); //constructor

    void initialise(int startX, int startY, int goalX, int goalY);

    void computeShortestPath();
    void updateVertex(LpaStarCell* cell);
	double minValue(double g_, double rhs_);
    int maxValue(int v1, int v2);
	void calcKey(int x, int y);
    void calcKey(LpaStarCell *cell);
    double calc_H(int x, int y);
    double getCost(LpaStarCell* succ, LpaStarCell* u);
    void updateNeighbours(int x, int y, char type);
    bool inHeap(LpaStarCell* u);
    void updateHValues();
    void updateAllKeyValues();

    void printQueue(priority_queue<LpaStarCell* > pq);
    void initialPlanning();

    void finalPlanning();

    void printResults(){
        
         if(heuristic == CHEBYSHEV){
            strHeuristic = "CHEBYSHEV";
            // cout << "\nh= CHEBYSHEV" << endl;
         } else if (heuristic == EUCLIDEAN){
            strHeuristic = "EUCLIDEAN";
            // cout << "\nh= EUCLIDEAN" << endl;
         }

        cout << setw(12) << std::left <<  "LPA_STAR";
        cout << setw(12) << std::left << "," <<  strHeuristic;
        cout << setw(27) << std::left << "," <<  gridWorldName;
        cout << setw(20) << std::left << "," <<  strEpisode;

        // cout << setw(12) << std::left <<  "LPA_STAR" << setw(12) << std::left << "," <<  strHeuristic << setw(27) << std::left << "," <<  gridWorldName << setw(20) << std::left << "," <<  strEpisode; 
        cout << setprecision(6) << std::setfill(' ') << std::fixed << std::right << ' ' << setw(10) << "," <<  stateExpansions;
        cout << setprecision(6) << std::setfill(' ') << std::fixed << std::right << ' ' << setw(10) << "," <<  maxQLength;
        cout << setprecision(6) << std::setfill(' ') << std::fixed << std::right << ' ' << setw(10) << "," <<  vertexAccesses;
        cout << setprecision(6) << std::setfill(' ') << std::fixed << std::right << ' ' << setw(10) << "," <<  pathLength << endl;

    }

    friend void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa);
    friend void copyMazeToDisplayMapShortestPath(GridWorld &gWorld, LpaStar* lpa);
    friend void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa);

    int stateExpansions;    
    int maxQLength;
    int vertexAccesses;
    float pathLength;
    float runningTime;
    unsigned int heuristic;
    string strHeuristic;
    string gridWorldName;
    int episode;
    string strEpisode;

private:
	
    vector<vector<LpaStarCell> > maze;   
    LpaStarCell l;
    vector<LpaStarCell* > U;//Priority Queue
    vector<LpaStarCell* > PopOut; //Priority Queue
    LpaStarCell* start;
    LpaStarCell* goal;
    LpaStarCell* temp = nullptr;

    int tempStartY;
    int tempStartX;

    int vertextAccess = 0;
    int maxQLenght = 0;
    int rows;
    int cols;
    int current_top_coord;
};

#endif
