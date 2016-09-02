#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>

using std::vector ;
using std::string ;
using std::stringstream ;
using std::ifstream ;
using std::getline ;

class Voronoi
{
  public:
    Voronoi(const char *) ;
    ~Voronoi(){}
    
    int Membership(double, double) ; // output voronoi index
    bool MovingToNextVoronoi(double, double, int) ; // true if current cell is adjacent to cell of target voronoi
  private:
    int numRows ;
    int numCols ;
    int rOffset ;
    int cOffset ;
    double res ;
    vector< vector<int> > cells ; // first dimension maps to x, second dimension maps to y
    
    void WorldToCells(double, double, int&, int&) ; // convert odometry position to cell index
} ;

Voronoi::Voronoi(const char * filename){
  // Read in parameters
  ros::param::get("rows", numRows);
  ros::param::get("columns", numCols);
  ros::param::get("cell_offset/x", rOffset);
  ros::param::get("cell_offset/y", cOffset);
  ros::param::get("resolution", res);

  // Read in voronoi map  
  ifstream mapFile(filename) ;

	ROS_INFO("Reading map from file...") ;
	vector<int> v(numCols) ;
	string line ;
	while (getline(mapFile,line)){
		stringstream lineStream(line) ;
		string cell ;
		int i = 0 ;
		while (getline(lineStream,cell,','))
			v[i++] = atoi(cell.c_str()) ;
		cells.push_back(v) ;
	}
	ROS_INFO("Voronoi map upload complete!") ;
}

int Voronoi::Membership(double x, double y){
  int mx ;
  int my ;
  WorldToCells(x,y,mx,my) ;
  return cells[mx][my] ;
}

bool Voronoi::MovingToNextVoronoi(double x, double y, int nextV){
  int mx ;
  int my ;
  WorldToCells(x,y,mx,my) ;
  int curV = cells[mx][my] ;
  
  // Compute valid neighbour cells
  vector<int> v(2) ;
  vector< vector<int> > nCells ;
  v[0] = mx ; // check left and right cells
  if (my > 0){
    v[1] = my-1 ;
    nCells.push_back(v) ;
  }
  if (my < numCols-1){
    v[1] = my+1 ;
    nCells.push_back(v) ;
  }
  if (mx > 0){ // check cells in front
    v[0] = mx-1 ;
    v[1] = my ;
    nCells.push_back(v) ;
    if (my > 0){
      v[1] = my-1 ;
      nCells.push_back(v) ;
    }
    if (my < numCols-1){
      v[1] = my+1 ;
      nCells.push_back(v) ;
    }
  }
  if (mx < numRows-1){ // check cells behind
    v[0] = mx+1 ;
    v[1] = my ;
    nCells.push_back(v) ;
    if (my > 0){
      v[1] = my-1 ;
      nCells.push_back(v) ;
    }
    if (my < numCols-1){
      v[1] = my+1 ;
      nCells.push_back(v) ;
    }
  }
  
  // Return true if next voronoi begins in any of the <=8 neighbouring cells
  for (size_t i = 0; i < nCells.size(); i++)
    if (cells[nCells[i][0]][nCells[i][1]] == nextV)
      return true ;
  return false ;
}

void Voronoi::WorldToCells(double x, double y, int& mx, int& my){
  mx = (int)round(x/res) ;
  my = (int)round(y/res) ;
  mx += rOffset ;
  my += cOffset ;
}
