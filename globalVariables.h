#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__


#include <vector>

/*******************************************************************************************************************/
//------------------------------------------
//Flags - enable or disable features


#define INCREMENTAL_SEARCH_ALGORITHM
#define RL_ALGORITHM

#define EIGHT_CONNECTED_GRIDWORLD
//#define FOUR_CONNECTED_GRIDWORLD


#define CHEBYSHEV	1
#define EUCLIDEAN	2

#define DEBUG false//true

//------------------------------------------


//-------------------------------------------------------------------------------
#ifdef FOUR_CONNECTED_GRIDWORLD
 
	//4-connected gridworld
	#define DIRECTIONS 4
	const struct {
	  int x;
	  int y;
	} neighbours[4]={{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

	/////////////////////////////////////////////////////

#endif

const double SQRT_2 =  1.4142135623731;

//-------------------------------------------------------------------------------
#ifdef EIGHT_CONNECTED_GRIDWORLD

	//8-connected gridworld
	#define DIRECTIONS 8
	
	//movement sequence, used in the journal
	const struct {
	  int x;
	  int y;
	} neighbours[8]={ {-1,-1}, {0, -1}, {1, -1}, 
					{-1, 0}, {1, 0}, 
					{-1, 1}, {0, 1}, {1, 1} };
			
	//clockwise, starting at 3 o'clock			
	//~ const struct {
	  //~ int x;
	  //~ int y;
	//~ } succ[8]={ {1,0}, {1, 1}, {0,1}, {-1, 1}, {-1, 0}, {-1,-1}, {0, -1}, {1, -1} };
		
#endif
//-------------------------------------------------------------------------------

//------------------------------------------
	
extern int numberOfExpandedStates;
extern int numberOfVertexAccesses;
extern int maxQLength;	
extern int qLengthAfterSearch;
	
extern bool MAP_INITIALISED;
extern bool PRECALCULATED_GRIDWORLD_READY;

//~ extern bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;
extern unsigned int HEURISTIC;

//Robot soccer dimensions	
extern int GRIDWORLD_ROWS; 
extern int GRIDWORLD_COLS;

//////////////////////////////////////////////////////////////////////////////
//REINFORCEMENT LEARNING

//extern int MAX_ACTIONS;

//end_REINFORCEMENT LEARNING
//////////////////////////////////////////////////////////////////////////////
using namespace std;

enum cellType{TRAVERSABLE=0, BLOCKED=1, UNKNOWN=9};
enum vertexStatus{UNEXPLORED=0, EXPANDED=1, ACCESSED=2};

 struct CellPosition
{
	int row;
	int col;
};

 struct Coordinates
{
	int x, y;
};


typedef struct {
  int y;
  int x;
} loc_t;


struct vertex
{
	
#ifdef INCREMENTAL_SEARCH_ALGORITHM	
    double rhs;
    double g;
	 int c;
	 double h;
	 double f;
	 double key[2];
	 vertex* move[DIRECTIONS]; 
    double linkCost[DIRECTIONS];	
#endif	
	
	
	
	
#ifdef RL_ALGORITHM
	
	#ifdef EIGHT_CONNECTED_GRIDWORLD
    double Q[8];
		
	#elif FOUR_CONNECTED_GRIDWORLD
    double Q[4];
	#endif
	
	 double sumQ;
	 double maxQ;
	
	 int reward;
#endif	
	
	 //--------------------------------------------------------------------------------- 
	 //TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
    char type; 
	 //---------------------------------------------------------------------------------
	 int row;
	 int col;
	 char status; 
	 
	 int x1,y1,x2,y2;
	 Coordinates centre; //centre x, centre y
}; 

extern int MAX_MOVES;

extern vector<vector<vertex> > map;
extern vertex startVertex;
extern vertex goalVertex;



typedef struct vector<CellPosition> PathType;

/*******************************************************************************************************************/
extern int fieldX1, fieldY1, fieldX2, fieldY2; //playing field boundaries
extern float WORLD_MAXX;
extern float WORLD_MAXY;
// colour constants
extern int BACKGROUND_COLOUR;
extern int LINE_COLOUR;

extern int cellWidth;
extern int cellHeight;

/********************************************************************************************************************/
//8-connected gridworld
#define DIRECTIONS 8 //clockwise sequence of moves (8-connected gridworld)

#define INF  1000000




struct LpaStarCell
{
    LpaStarCell* move[DIRECTIONS];
    LpaStarCell* predecessor[DIRECTIONS];
	LpaStarCell* successor[DIRECTIONS];
	 double linkCost[DIRECTIONS];
    LpaStarCell* parent;
    LpaStarCell* trace;
    short obstacle;
    int x, y;

	 double g;
    double rhs;
	 double h;
    double key[2];
	
	 //~ int g;
    //~ int rhs;
    //~ int key[2];
	
	
	 //---------------------
	 //TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	 char type;
	 //----------------------
	
	
	
    int generated;
    int heapindex;

};

// comparator for priority queue 
struct LpaStarCellComparator{
	bool operator()(const LpaStarCell* a, const LpaStarCell* b) const {
		
		bool result = false;
		if(a->key[0] < b->key[0]){
			result = true;
		}else if(a->key[0] == b->key[0]){
			if(a->key[1] < b->key[1]){
				result = true;
			}
		}
        return result; // Min-heap: Compare based on 'x' in ascending order
    }
};


// comparator for shortest path
struct LpaStarCellComparatorShortestPath{
	bool operator()(const LpaStarCell* e1, const LpaStarCell* e2) const {
		bool val = false;
		if (e1->key[0] == e2->key[0]) {
			if (e1->key[1] == e2->key[1]) {
				if (e1->g == e2->g) {
					if (e1->rhs == e2->rhs) {
						val = e1->rhs < e2->rhs; // Sort by h in ascending order if g and rhs are equal
					}
					// return e1->rhs < e2->rhs; // Sort by rhs in ascending order if g is equal
				}
				// return e1->g < e2->g; // Sort by g in ascending order
			}else{
				val = e1->key[1] > e2->key[1]; // Sort by key[1] in descending order
			}        
		}else{
			val = e1->key[0] > e2->key[0]; // Sort by key[0] in descending order
		}
		return val;    
	}
};


extern bool SHOW_MAP_DETAILS;

/********************************************************************************************************************/



#endif
