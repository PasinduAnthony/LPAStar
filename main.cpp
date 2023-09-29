///////////////////////////////////////////////////////////////////////////////////////////
//                      
//
//	 	      Program Name: Incremental Search 
//	 	       Description: start-up code for simulating LPA* and D*Lite
//                        - implements a gridworld that can be loaded from file, and 
//                          modified through a user-interface 
//
//        Run Parameters: 
//
//    Keys for Operation: 
//
//	 		        History:  date of revision
//                         06/Sept/2023
//
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////

#include <windows.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <deque>
#include <set>
#include <vector>
#include <chrono>

//-------------------------
#include "globalVariables.h"
#include "transform.h"
#include "graphics.h"
#include "AstarSearch.h"
#include "LPAstar.h"
#include "gridworld.h"



// colour constants
int BACKGROUND_COLOUR;
int LINE_COLOUR;

int robotWidth;
int GRIDWORLD_ROWS; //duplicated in GridWorld
int GRIDWORLD_COLS; //duplicated in GridWorld


//----------------------------
unsigned int HEURISTIC;
//~ bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;
int numberOfExpandedStates;
int MAX_MOVES;
int maxQLength;
int qLengthAfterSearch;

///////////////////////////////////////////////////////////////////////////////
LpaStar* lpa_star;
GridWorld grid_world;

bool SHOW_MAP_DETAILS;
bool CONTROL_KEY_FLAG;
///////////////////////////////////////////////////////////////////////////////

string list_of_grid_worlds[6] = {"grid_dstar_journal", "grid_lpa_journal", "grid_lpa_journal_big", "grid_spiral", "grid_trap", "grid_big"};

///////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------
//copy maze (from LPA*) to map (of GridWorld)
void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa){
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			gWorld.map[i][j].type = lpa->maze[i][j].type;
		   gWorld.map[i][j].h = lpa->maze[i][j].h;
			gWorld.map[i][j].g = lpa->maze[i][j].g;
			gWorld.map[i][j].rhs = lpa->maze[i][j].rhs;
			gWorld.map[i][j].row = lpa->maze[i][j].y;
			gWorld.map[i][j].col = lpa->maze[i][j].x;
			
			for(int k=0; k < 2; k++){
			  gWorld.map[i][j].key[k] = lpa->maze[i][j].key[k];			  
			}
			
			
		}
	}
	gWorld.map[lpa->start->y][lpa->start->x].h = lpa->start->h;
	gWorld.map[lpa->start->y][lpa->start->x].g = lpa->start->g;
	gWorld.map[lpa->start->y][lpa->start->x].rhs = lpa->start->rhs;
	gWorld.map[lpa->start->y][lpa->start->x].row = lpa->start->y;
	gWorld.map[lpa->start->y][lpa->start->x].col = lpa->start->x;
	for(int k=0; k < 2; k++){
			  gWorld.map[lpa->start->y][lpa->start->x].key[k] = lpa->start->key[k];			  
	}
	
	
	gWorld.map[lpa->goal->y][lpa->goal->x].h = lpa->goal->h;
	gWorld.map[lpa->goal->y][lpa->goal->x].g = lpa->goal->g;
	gWorld.map[lpa->goal->y][lpa->goal->x].rhs = lpa->goal->rhs;
	gWorld.map[lpa->goal->y][lpa->goal->x].row = lpa->goal->y;
	gWorld.map[lpa->goal->y][lpa->goal->x].col = lpa->goal->x;
	for(int k=0; k < 2; k++){
			  gWorld.map[lpa->goal->y][lpa->goal->x].key[k] = lpa->goal->key[k];			  
	}
	
}

//--------------------------------------------------------------
//copy map (of GridWorld)to maze (of LPA*)
void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa){
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			lpa->maze[i][j].type = gWorld.map[i][j].type;
			lpa->maze[i][j].x = gWorld.map[i][j].col;
			lpa->maze[i][j].y = gWorld.map[i][j].row;
			
		   //lpa->maze[i][j].g = gWorld.map[i][j].g;
			//lpa->maze[i][j].rhs = gWorld.map[i][j].rhs;
		}
	}
	
	vertex startV = gWorld.getStartVertex();
	vertex goalV = gWorld.getGoalVertex();
	
	//lpa->start->g = gWorld.map[startV.row][startV.col].g ;
	//lpa->start->rhs = gWorld.map[startV.row][startV.col].rhs ;
	lpa->start->x = gWorld.map[startV.row][startV.col].col;
	lpa->start->y = gWorld.map[startV.row][startV.col].row;
	
	//lpa->goal->g = gWorld.map[goalV.row][goalV.col].g;
	//lpa->goal->rhs = gWorld.map[goalV.row][goalV.col].rhs;
	lpa->goal->x = gWorld.map[goalV.row][goalV.col].col;
	lpa->goal->y = gWorld.map[goalV.row][goalV.col].row;
	
}


///////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES


///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////




void drawInformationPanel(int x, int y, char* info){
   ///////////////////////////////////////////////////////////////////////////////////////////
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,CENTER_TEXT);
	setcolor(YELLOW);
	if (info != NULL) {
		outtextxy(x, y, info);
	}
		
	///////////////////////////////////////////////////////////////////////////////////////////
}

int getKey(){
	
	 // if(GetAsyncKeyState(VK_UP) < 0) { //UP ARROW
  //      return 2000;
  //   } 
	 
	 // if(GetAsyncKeyState(VK_DOWN) < 0) { //DOWN ARROW
  //      return 2001;
  //   }
	
    if(GetAsyncKeyState(VK_F4) < 0) { 
       SHOW_MAP_DETAILS=false;
		 return 104;
    } 
  
    if(GetAsyncKeyState(VK_F5) < 0) {
        SHOW_MAP_DETAILS=true;
		  return 105;
    }
	 
	 if(GetAsyncKeyState(VK_F6) < 0) {
        //execute A* with strict expanded list
		  return 106;
    }
	 if(GetAsyncKeyState(VK_F7) < 0) {
        //execute LPA*
		  return 107;
    }
	 if(GetAsyncKeyState(VK_F8) < 0) {
        //execute D*Lite
		  return 108;
    }
	 
	 //copy display map to algorithm's maze
	 if(GetAsyncKeyState(VK_F9) < 0) {
	 	if(CONTROL_KEY_FLAG){
	 		return 109;
	 	}
		  
    }
	 
	 //copy algorithm's maze to display map
	 if(GetAsyncKeyState(VK_F10) < 0) {
		  return 110;
    }

    if(GetAsyncKeyState(VK_F12) < 0) {
		  if(CONTROL_KEY_FLAG){
    	  	   return 19;
    	  }
    }

    if(GetAsyncKeyState(0x54) < 0) { //T-key (Test)
		  if(CONTROL_KEY_FLAG){
    	  	   return 22;
    	  }
    }
	 	 
	 if(GetAsyncKeyState(0x53) < 0) { //S-key (start cell)
		  return 6;
    }
	 
	 if(GetAsyncKeyState(0x58) < 0) { //X-key (goal cell)
		  return 7;
    }
	 
	 if(GetAsyncKeyState(0x42) < 0) { //B-key (block cell)
	 	  // cout << "block cell." << endl;
		  return 1;
    }
	 
	 if(GetAsyncKeyState(0x47) < 0) {  //G-key
		  return 9;
    }
	 
	 if(GetAsyncKeyState(0x48) < 0) {  //H-key
		  return 10;
    }
	 
	 if(GetAsyncKeyState(0x4B) < 0) {  //K-key
		  return 11;
    }
	 
	 if(GetAsyncKeyState(0x55) < 0) { //U-key (Unblock cell)
		  // cout << "unblock cell." << endl;
		  return 12;
    }
	 
	 if(GetAsyncKeyState(0x50) < 0) { //P-key (position of cells)
		  return 14;
    }
	 
	 if(GetAsyncKeyState(0x43) < 0) { //C-key (connections of cells)
		// if(CONTROL_KEY_FLAG){
		//    CONTROL_KEY_FLAG=false;
		   return 15;
		// }
    }
	 
	 if(GetAsyncKeyState(0x4D) < 0) { //M-key (entire map connections)
		  return 16;
    }

    if(GetAsyncKeyState(VK_CONTROL) < 0) { //Ctrl key
   	 CONTROL_KEY_FLAG=true;
   	 
       return 111;
    }

    if(GetAsyncKeyState(VK_SPACE) < 0) { //Initial planning
		 if(CONTROL_KEY_FLAG){
		   cout << "Initial planning..." << endl; 
    	   return 100;	
		 }		 
    	  
    }

    if(GetAsyncKeyState(VK_RETURN) < 0) { //Re-planning
		 if(CONTROL_KEY_FLAG){
		   cout << "Re-planning..." << endl; 
    	   return 200;	
		 }		 
    	  
    }

    if(GetAsyncKeyState(0x53) & 0x8000) { //Ctrl + F12-key (save map)
    
    	  if(CONTROL_KEY_FLAG){
    	  	   return 19;
    	  }
		   
    }

    if(GetAsyncKeyState(0x59) < 0) { //Y-key (set as Type '9', Unknown to be blocked cell)
    	  cout << "unknown to be blocked cell (Type 9)." << endl;
		  return 20;
    }
    if(GetAsyncKeyState(0x5A) < 0) { //Z-key (set as Type '8', Unknown to be traversable cell)
    	  cout << "unknown to be traversable cell (Type 8)." << endl;
		  return 21;
    }
	 
	 
	 return 0;
 }
 
///////////////////////////////////////////////////////////////////////////////////////
 void runSimulation(char *fileName){
	WorldBoundaryType worldBoundary; //duplicated in GridWorld
   DevBoundaryType deviceBoundary; //duplicated in GridWorld
	bool ANIMATE_MOUSE_FLAG=false;
	bool validCellSelected=false;
	bool blockedCellSelected=false;
	static BOOL page=false;
	int mX, mY;
	float worldX, worldY;
	worldX=0.0f;
	worldY=0.0f;

	mX = 0; 
	mY = 0;
	

	//-----------------------
	CellPosition p;
	int rowSelected, colSelected;
	//-----------------------
   rowSelected=-1;
	colSelected=-1;
	
	int mouseRadius=1;
		
			
	//Initialise the world boundaries
   grid_world.initSystemOfCoordinates();
	grid_world.loadMapAndDisplay(fileName);
	grid_world.initialiseMapConnections();
	
	//----------------------------------------------------------------
	//LPA*
	lpa_star = new LpaStar(grid_world.getGridWorldRows(), grid_world.getGridWorldCols(), HEURISTIC, string(fileName) );
	vertex start = grid_world.getStartVertex();
	vertex goal = grid_world.getGoalVertex();
	
	cout << "(start.col = " << start.col << ", start.row = " << start.row << ")" << endl;
	cout << "(goal.col = " << goal.col << ", goal.row = " << goal.row << ")" << endl;
	
	lpa_star->initialise(start.col, start.row, goal.col, goal.row);
	
	copyDisplayMapToMaze(grid_world, lpa_star);

	lpa_star->computeShortestPath();	
	//----------------------------------------------------------------
		
	worldBoundary = grid_world.getWorldBoundary();
	deviceBoundary = grid_world.getDeviceBoundary();
	GRIDWORLD_ROWS = grid_world.getGridWorldRows();
	GRIDWORLD_COLS = grid_world.getGridWorldCols();
	
	int iter=0;
	int action=-1;
	int prevAction= -1;

	setactivepage(page);
	cleardevice();

   action = getKey(); 

   if(SHOW_MAP_DETAILS) {
   	grid_world.displayMapWithDetails();
   } else {
   	grid_world.displayMap();
   }
	setvisualpage(page);
	
	// keep running the program until the ESC key is pressed   
	while((GetAsyncKeyState(VK_ESCAPE)) == 0 ) {
	
			 prevAction = action;
		    action = getKey(); //get keyboard input
			 
			 //Process action selected
		    if( (action != prevAction) && action != -1){

		    	 cout << "\n prevAction = " << prevAction << endl;
		    	 cout << "\n action = " << action << endl;

				 switch(action){
					case 1: //Block selected cell
						{
							// retrieve current start vertex
                     vertex s = 	grid_world.getStartVertex();
                     // retrieve current GOAL vertex
					      vertex g = 	grid_world.getGoalVertex();
					      
                     cout << "(current start.col = " << s.col << ", current start.row = " << s.row << ")" << endl;
                     cout << "(current goal.col = " << g.col << ", current goal.row = " << g.row << ")" << endl;
                     if(rowSelected != -1 && colSelected != -1){
                       cout << "\n(row = " << rowSelected-1 << ", col = " << colSelected-1 << ")" << endl;  	
                     }
                     
					 		
                     if( rowSelected != -1 && colSelected != -1){
                     	if( !(((rowSelected-1) == s.row) && ((colSelected-1) == s.col))){
                     		if( !(((rowSelected-1) == g.row) && ((colSelected-1) == g.col))){
										grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '1');
										grid_world.initialiseMapConnections(); 
										
										rowSelected=-1;
										colSelected=-1;
								   }
								} 
							} 
						}

						break;

					case 12: //make cell Traversable
				 
						 {
							// retrieve current start vertex
                     vertex s = 	grid_world.getStartVertex();
                     // retrieve current GOAL vertex
					      vertex g = 	grid_world.getGoalVertex();
					      
                     cout << "(current start.col = " << s.col << ", current start.row = " << s.row << ")" << endl;
                     cout << "(current goal.col = " << g.col << ", current goal.row = " << g.row << ")" << endl;
                     if(rowSelected != -1 && colSelected != -1){
                       cout << "\n(row = " << rowSelected-1 << ", col = " << colSelected-1 << ")" << endl;  	
                     }
                     
					 		
                     if( rowSelected != -1 && colSelected != -1){
                     	if( !(((rowSelected-1) == s.row) && ((colSelected-1) == s.col))){
                     		if( !(((rowSelected-1) == g.row) && ((colSelected-1) == g.col))){										
										grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '0');
										grid_world.initialiseMapConnections(); 
										
										rowSelected=-1;
										colSelected=-1;
								   }
								} 
							} 
						}
						 
						break; 	

					case 20: //set cell as Type '9'

						{
							// retrieve current start vertex
                     vertex s = 	grid_world.getStartVertex();
                     // retrieve current GOAL vertex
					      vertex g = 	grid_world.getGoalVertex();
				 
						 if( rowSelected != -1 && colSelected != -1){
						 	if( !(((rowSelected-1) == s.row) && ((colSelected-1) == s.col))){
                     		if( !(((rowSelected-1) == g.row) && ((colSelected-1) == g.col))){	
							 			grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '9');
							 			grid_world.initialiseMapConnections(); 
							 			rowSelected=-1;
							         colSelected=-1;
							 		}	
							}
							 
						 }
						}
						
						break; 

	           case 21: //set cell as Type '8'

	           		{
							// retrieve current start vertex
                     vertex s = 	grid_world.getStartVertex();
                     // retrieve current GOAL vertex
					      vertex g = 	grid_world.getGoalVertex();
				 
						 if( rowSelected != -1 && colSelected != -1){
						 	if( !(((rowSelected-1) == s.row) && ((colSelected-1) == s.col))){
                     		if( !(((rowSelected-1) == g.row) && ((colSelected-1) == g.col))){	
							 			grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '8');
							 			grid_world.initialiseMapConnections(); 
							 			rowSelected=-1;
							         colSelected=-1;
							 		}	
							}
							 
						 }
						}
				 
						 break; 	

					
					case 105: 
						   
						   

						   if(grid_world.isGridMapInitialised()){
						   	cout << "\nletter F5, pressed." << endl;
						   	page = !page; //new - napoleon; this fixed it!
						   	setactivepage(page);

						   	if(SHOW_MAP_DETAILS) {
							      grid_world.displayMapWithDetails();
							   } else {
							      grid_world.displayMap();
							   }
								
								setvisualpage(page);
								
                        while(GetAsyncKeyState(VK_F5) != 0){
                        	//wait for letter F5 to be released
                        }
								
							} else {
							 	cout << "map has not been initialised yet." << endl;
								break;
							}
							SHOW_MAP_DETAILS=false;
							action = -1;
						   
							break;
					
					case 106: 
						  
					
							break;
					
					case 107: 
						  
					
							break;
					
					case 108: 
						  
					
							break;
					
					case 15:
						 
						   if(grid_world.isGridMapInitialised()){

								   if( validCellSelected && rowSelected != -1 && colSelected != -1 && !blockedCellSelected){
								   
								   	cout << "\nletter c, pressed." << endl;
								   	page = !page; //new - napoleon; this fixed it!
								   	setactivepage(page);		

								   	if(SHOW_MAP_DETAILS) {
									      grid_world.displayMapWithDetails();
									   } else {
									      grid_world.displayMap();
									   }		
										grid_world.displayVertexConnections(colSelected-1, rowSelected-1);
									   setvisualpage(page);
									   
									   rowSelected=-1;
									   colSelected=-1;

									   
							         while(GetAsyncKeyState(0x43) != 0){
                        	       //wait for letter c to be released
                              }
									   
									   
								   } 
							   } else {
									cout << "please select a valid cell first." << endl;
									break;
							   }

						   
							if(validCellSelected && !blockedCellSelected){
								rowSelected=-1;
							   colSelected=-1;
							   action = -1;							   
						   }
							
						   
						   break;
							
						
					
					case 16:
						 
						   if(grid_world.isGridMapInitialised()){
						   	cout << "\nletter m, pressed." << endl;
						   	page = !page; //new - napoleon; this fixed it!
						   	setactivepage(page);

						   	if(SHOW_MAP_DETAILS) {
							      grid_world.displayMapWithDetails();
							   } else {
							      grid_world.displayMap();
							   }
								grid_world.displayMapConnections();
								setvisualpage(page);
								
                        while(GetAsyncKeyState(0x4D) != 0){
                        	//wait for letter m to be released
                        }
								
							} else {
							 	cout << "map has not been initialised yet." << endl;
								break;
							}
							
				
							//--------------------------------------------
	
						   break;		

					case 6: //set cell as new START vertex 
					   {

						   //--------------------------------------------
					      // retrieve current START vertex
					      vertex s = 	grid_world.getStartVertex();
					      cout << "(current start.col = " << s.col << ", current start.row = " << s.row << ")" << endl;
					      if( (s.row != -1) && (s.col != -1) ){
								//set current START VERTEX to an ordinary TRAVERSABLE CELL
								grid_world.setMapTypeValue(s.row, s.col, '0'); 
								grid_world.initialiseMapConnections(); 
								
							} else {
								cout << "invalid START vertex" << endl;
								break;
							}
					      //--------------------------------------------
							//set selected cell as the NEW START VERTEX
						   if( rowSelected != -1 && colSelected != -1){
							   grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '6');
							   s.row = rowSelected-1;
								s.col = colSelected-1;
								grid_world.setStartVertex(s);
								
							   rowSelected=-1;
							   colSelected=-1;

						   } else {
								cout << "invalid new START vertex, please select a new START vertex first." << endl;
								break;
							}
							//--------------------------------------------
						   
							
						}
						break;
					
					case 7: //set cell as new GOAL vertex 
					   {
						   //--------------------------------------------
					      // retrieve current GOAL vertex
					      vertex s = 	grid_world.getGoalVertex();
					      if( (s.row != -1) && (s.col != -1) ){
								//set current GOAL VERTEX to an ordinary TRAVERSABLE CELL
								grid_world.setMapTypeValue(s.row, s.col, '0'); 
								
								//ok, proceed
							} else {
								cout << "invalid GOAL vertex" << endl;
								action = -1;
								break;
							}
					      //--------------------------------------------
							//set selected cell as the NEW GOAL VERTEX
						   if( rowSelected != -1 && colSelected != -1){
							   grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '7');
							   s.row = rowSelected-1;
								s.col = colSelected-1;
								grid_world.setGoalVertex(s);
								grid_world.initialiseMapConnections(); 
								
							   rowSelected=-1;
							   colSelected=-1;

						   } else {
								cout << "invalid new GOAL vertex, please select a new GOAL vertex first." << endl;
								action = -1;
								break;
							}
							//--------------------------------------------
						   
							
						}
						break;
								
	            case 22:	//Test			 
	                  if(CONTROL_KEY_FLAG)	{
		                  CONTROL_KEY_FLAG=false;
		                  copyDisplayMapToMaze(grid_world, lpa_star);
						      cout << "copied display map to algorithm's maze" << endl;					      
	                  }
						   // action = -1; //new - napoleon; we want the program to display the grid again
					      break;
					
					case 100:	//Initial planning
	                  if(CONTROL_KEY_FLAG)	{
		                  CONTROL_KEY_FLAG=false;
		                  copyDisplayMapToMaze(grid_world, lpa_star);

		                  //initialise algorithm
		                  //path = compute shortest path
		                  // etc.
						      
						      //
						      //To do:	      
						      //  you need to add more statements here to allow for initial planning
						      //
	                  }
						   // action = -1; //new - napoleon; we want the program to display the grid again
					      break;
					
	            case 200:	//Replanning
	                  if(CONTROL_KEY_FLAG)	{
		                  CONTROL_KEY_FLAG=false;
		                  
						      
						      //
						      //To do:	      
						      //  you need to add more statements here to allow for Re-planning
						      //
						      copyMazeToDisplayMap(grid_world, lpa_star);
	                  }
						   // action = -1; //new - napoleon; we want the program to display the grid again
					      break; 

					case 110:					
						   // lpa_star->updateHValues();
						   // copyMazeToDisplayMap(grid_world, lpa_star);
					      // cout << "copied algorithm's maze to display map" << endl;
					      action = -1;
					      break;
					
					case 9: //display g-values only
						   cout << "\nletter g, pressed." << endl;
						   page = !page; //new - napoleon; this fixed it!
						   setactivepage(page);

						   if(SHOW_MAP_DETAILS) {
							   grid_world.displayMapWithDetails();
							} else {
							   grid_world.displayMap();
							}
							grid_world.displayMapWithSelectedDetails(true, false, false, false);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
							setvisualpage(page);
							
                     while(GetAsyncKeyState(0x47) != 0){
                       	//wait for letter g to be released
                     }
						   
					      action = -1;
							break;
	            case 10: //display h-values only
	            	   cout << "\nletter h, pressed." << endl;
						   page = !page; //new - napoleon; this fixed it!
						   setactivepage(page);

						   if(SHOW_MAP_DETAILS) {
							   grid_world.displayMapWithDetails();
							} else {
							   grid_world.displayMap();
							}
							grid_world.displayMapWithSelectedDetails(false, false, true, false);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
							setvisualpage(page);
							
                     while(GetAsyncKeyState(0x48) != 0){
                       	//wait for letter h to be released
                     }
						   
					 		action = -1;
					      break;
					case 11: //display key-values only

						   cout << "\nletter k, pressed." << endl;
						   page = !page; //new - napoleon; this fixed it!
						   setactivepage(page);

						   if(SHOW_MAP_DETAILS) {
							   grid_world.displayMapWithDetails();
							} else {
							   grid_world.displayMap();
							}
							grid_world.displayMapWithSelectedDetails(false, false, false, true);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
							setvisualpage(page);
							
                     while(GetAsyncKeyState(0x4B) != 0){
                       	//wait for letter k to be released
                     }
						   
					 		action = -1;
						   // lpa_star->updateAllKeyValues();
					      // copyMazeToDisplayMap(grid_world, lpa_star);
						   
							
					      break;
					
					
						 
					case 14: //p - key; 0x50
						   cout << "\nletter p, pressed." << endl;
						   page = !page; //new - napoleon; this fixed it!
						   setactivepage(page);

						   if(SHOW_MAP_DETAILS) {
							   grid_world.displayMapWithDetails();
							} else {
							   grid_world.displayMap();
							}
							grid_world.displayMapWithPositionDetails();
							setvisualpage(page);
							
                     while(GetAsyncKeyState(0x50) != 0){
                       	//wait for letter p to be released
                     }						   
					 		action = -1;					   
					      break;	 
						 
					case 19: 
						   if(CONTROL_KEY_FLAG){
	                     CONTROL_KEY_FLAG = false;

						   	string targetFileName;
	                     cout << "Save map to file." << endl;
						   	cout << "Enter destination filename: ";
						   	cin >> targetFileName;
	                     grid_world.saveNewMap(targetFileName.c_str());
	                     
						   }
						   
							action = -1;
					      break;  
					
			  
					  
					 
			    };
			   //---
			   setactivepage(page);
			   cleardevice();
			   grid_world.displayMap();

			   // if(SHOW_MAP_DETAILS) {
			   //    grid_world.displayMapWithDetails();
			   // } else {
			   //    grid_world.displayMap();
			   // }
			   setvisualpage(page);
			   page = !page; 
			   cout << "\niter = "  << iter << endl;
			   iter++;

		 }
		
		//----------------------------------------------------------------------------------------------------------------

      //To do: add statements to display the computed shortest path here 
      // if(path is Found){
      // 	display path
      // }

	   //----------------------------------------------------------------------------------------------------------------	  
		// Mouse handling
		//
		    ANIMATE_MOUSE_FLAG=false; //new
			 if(!SHOW_MAP_DETAILS){
				 if(mousedown()){
							 				
					ANIMATE_MOUSE_FLAG=true;
					 			 
					mX = mousecurrentx();
					mY = mousecurrenty();
					 
					//if the goal selected is within the playing field boundaries
					if(mX >= grid_world.getFieldX1() && mX <= grid_world.getGridMaxX() && mY >= grid_world.getFieldY1() && mY <= grid_world.getGridMaxY()){
						    blockedCellSelected=false;
						    circle(mX, mY, 3);
						    validCellSelected = true;
                      p=grid_world.getCellPosition_markCell(mX, mY);
							 rowSelected = p.row;
							 colSelected = p.col;
							 char tp = grid_world.getMapTypeValue(rowSelected-1, colSelected-1);
                      cout << "\n(row = " << rowSelected << ", col = " << colSelected << endl;  
							 if(tp == '1'){
							  	 blockedCellSelected=true;
							  	 cout << "\nblocked cell selected." << endl;
							 }

	  	            
					} else {
						validCellSelected = false;
					}
				 } //end of mousedown()
			 }
			 //------------------------------------------------------------------------------------------------------------------
			 /////////////////////////////////////////////////////////////////////////////
			 						 
			 while(ANIMATE_MOUSE_FLAG)	 {
				if(mouseRadius < 40) {
					//mouseRadius += 3;
					mouseRadius += 1;
				}
				if(mouseRadius >= 40) {
					ANIMATE_MOUSE_FLAG=false;
					mouseRadius=0;
				}
				setactivepage(page);
				cleardevice();
		
			 						
			 	grid_world.displayMap();
			 						 
			   if(ANIMATE_MOUSE_FLAG){
					
				  //draw Cross-hair to mark Goal	    
				  setcolor(RED);
				  circle(mX, mY, 20);
				  line(mX,mY-20,mX,mY+20);
				  line(mX-20,mY,mX+20,mY);
				  //end of draw Cross-hair 
			 
				  // special effect to display concentric circles locating the target
					setcolor(YELLOW);					
					circle(mX, mY, mouseRadius);					
					//end of special effect
			   }

			  char info[256]; 
			  float wX, wY;
			  
			  strcpy(info, "");

			  
			  if(validCellSelected) {
				  p=grid_world.getCellPosition_markCell(mX, mY);
				  rowSelected = p.row;
				  colSelected = p.col;


				  strcpy(info, "");
				  snprintf(info,sizeof(info),"row: %d, col: %d",rowSelected, colSelected); 
			     drawInformationPanel(grid_world.getFieldX2(),grid_world.getFieldY1() + textheight("H")*6, info);


			  }
			   setvisualpage(page);
			   page = !page;  //switch to another page


			} //while(ANIMATE_MOUSE_FLAG) - End
		// cout << "shortes path called" << endl;
		// lpa_star->computeShortestPath();	  
	}
}

///////////////////////////////////////////////////////////////////////////////////////
void runExperiments(){
   WorldBoundaryType worldBoundary; //duplicated in GridWorld
   DevBoundaryType deviceBoundary; //duplicated in GridWorld

	static bool page=false;
	SHOW_MAP_DETAILS=false;

	float worldX, worldY;
	worldX=0.0f;
	worldY=0.0f;

//Perform all experiments
 	   
  	int num_of_grid_worlds = sizeof(list_of_grid_worlds)/ sizeof(list_of_grid_worlds[0]);

         for(int h=1; h <= 2; h++){ //heuristics

            for(int j=0; j < num_of_grid_worlds; j++){

            	grid_world.initSystemOfCoordinates();

            	char fName[128];

            	strcpy(fName, (char*)list_of_grid_worlds[j].c_str());
            	strcat(fName,".map");

				string path_and_fName; 
               
               path_and_fName = "./grids/" + string(fName); 
               
               if(DEBUG){
                 cout << "\npath_and_fName = " << path_and_fName << endl;
               }
			        SHOW_MAP_DETAILS=false;

					grid_world.loadMapAndDisplay((const char *) path_and_fName.c_str());					
					grid_world.initialiseMapConnections();

//----------------------------------------------------------------              

					//LPA*
					lpa_star = new LpaStar(grid_world.getGridWorldRows(), grid_world.getGridWorldCols(), h, fName); //here
					vertex start = grid_world.getStartVertex();
					vertex goal = grid_world.getGoalVertex();
					

					lpa_star->initialise(start.col, start.row, goal.col, goal.row);

            	    copyDisplayMapToMaze(grid_world, lpa_star);
            	    worldBoundary = grid_world.getWorldBoundary();
					deviceBoundary = grid_world.getDeviceBoundary();
					GRIDWORLD_ROWS = grid_world.getGridWorldRows();
					GRIDWORLD_COLS = grid_world.getGridWorldCols();
                    lpa_star->initialPlanning();

                    setactivepage(page);
					cleardevice();
				   
				   	grid_world.displayMap();
				   
				   setvisualpage(page);
				   page = !page;

                   lpa_star->printResults();
                   // cout << "\npress any key to continue..." << endl;
                   getch();
               

               // lpa_star->finalPlanning();
               // lpa_star->printResults();
               // cout << "\npress any key to continue..." << endl;
               // getch();
               delete lpa_star;

            }
      
      	
      }
      // cout << "\ndone." << endl;
}


///////////////////////////////////////////////////////////////////////////////////////

//search.exe grid_lpa_journal.map m
//search.exe ./grids/grid_lpa_journal.map m

int main(int argc, char *argv[]) {	
	char gridFileName[80];

	int graphDriver = 0,graphMode = 0; 	
	initgraph(&graphDriver, &graphMode, "", 1360, 768); // Start Window - LAPTOP SCREEN
	//initgraph(&graphDriver, &graphMode, "", 1920, 1080); // Start Window - Full-HD
	
	if (argc == 3){
        

	    string heuristic(argv[2]);
	    std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(),::tolower);
     
		strcpy(gridFileName, argv[1]);
		cout << "\ngridFileName = " << gridFileName << endl;
        
		//heuristic function selection
		if((heuristic.compare("euclidean")==0) || (heuristic.compare("e")==0)){
			HEURISTIC = EUCLIDEAN;
			cout << "Heuristic function = EUCLIDEAN" << endl;
		} else if((heuristic.compare("chebyshev")==0) || (heuristic.compare("c")==0)){
			HEURISTIC = CHEBYSHEV;
			cout << "Heuristic function = CHEBYSHEV" << endl;
		} else {
         cout << "Invalid parameters:  gridworld heuristic" << endl;
		   cout << "Example: ./main .\\grids\\grid_Dstar_journal.map m" << endl;
		}

		
		
		BACKGROUND_COLOUR = WHITE;
		LINE_COLOUR = GREEN;
		
		GRIDWORLD_ROWS = 0; //7; //6; //duplicated in GridWorld
	    GRIDWORLD_COLS = 0; //15;//13; //duplicated in GridWorld
		SHOW_MAP_DETAILS=false;
		CONTROL_KEY_FLAG=false;

	    try{
			runSimulation(gridFileName);
			// runSimulation_old(gridFileName);
	    }
		
	    catch(...){
	    	cout << "Exception caught!\n";
	    }

	} else if(argc == 2){
		   string run_mode(argv[1]);
		   std::transform(run_mode.begin(), run_mode.end(), run_mode.begin(),::tolower);
	   	if((run_mode.compare("all")==0) || (run_mode.compare("a")==0)){
	   		using std::chrono::system_clock;
            system_clock::time_point start;             
            start = std::chrono::system_clock::now();
	   		
            runExperiments();

            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
            string timeStr = to_string(elapsed_seconds.count()); 
            timeStr = timeStr + " sec.";
            cout << "\nTotal time = " << timeStr << endl;

	   	} else {
	   		cout << "Invalid parameter." << endl;		
	   		cout << "Example: ./main all" << endl;		
	   	}

	} else {
		cout << "\nmissing parameters!" << endl;		
	}
	

	cout << "----<< The End.>>----" << endl;
	
	return 0;
} 

