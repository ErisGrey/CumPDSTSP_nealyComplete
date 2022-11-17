#pragma once
#ifndef BENDERSOLVER_H
#define BENDERSOLVER_H
#include "instance.h"
#include <ilcplex/ilocplex.h>
#include <unordered_map>
#include <unordered_set>
//#include "graph.h"

#include <chrono>
ILOSTLBEGIN

typedef IloArray<IloArray<IloNumVarArray>> NumVar3D;
typedef IloArray<IloArray<IloNumArray>> Num3D;
typedef IloArray<IloNumVarArray> NumVar2D;
typedef IloArray<IloNumArray> Num2D;

typedef unordered_map<int, int> MapSol2D;
typedef vector<unordered_map<int, int>> MapSol3D;
typedef vector<tuple<int, int>> Sol2D;
typedef vector<tuple<int, int, int>> Sol3D;


using namespace std;


class Solver {
public:
    Instance* instance;
    IloArray<IloNumVarArray> x; // x_ij
    IloArray<IloNumVarArray> y; // y_kij
    IloArray<IloNumVarArray> z; // z_i
    IloNumVarArray u;
    IloArray<IloNumVarArray> v;
    NumVar2D f;
    IloNumVar Z;
    IloNumVar alpha;
    IloNumVarArray T;
    IloConstraintArray workerConstraints;

    IloObjective subObj;
    IloEnv env, workerEnv;
    IloCplex cplex, workerCplex;
    IloModel model, workerModel;
    double UB;
    double UB_tsp;
    vector<vector<int>> stour_tsp;
    vector<vector<int>> stour_drone;

    int numNode;
    int numUAV;

    vector<int> N;
    vector<int> C;
    unordered_set<int> truckOnly;
    unordered_set<int> freeCustomers;
    unordered_set<int> freeCustomers0;

    unordered_map<int, int> bestX;
    unordered_multimap<int, int> bestY;
    unordered_map<int, int> bestZ;
    //   vector<Tuple3> bestF;

    Solver(Instance*, string input, string output, double time_limit, double mem_limit);
    void Solve();
    //void plot();
    //double servable(int dc, int customer);
    //void rebuildWorkerLP(MapSol2D &xSol, MapSol2D &ySol, MapSol2D &zSol);
    //void updateSubtour_tsp(unordered_map<int, int>& xSol);
 //   void updateSubtour_tsp_lp(vector<Edge>& xSol);
 //   void updateSubtour_drone(unordered_multimap<int, int> ySol);
 //   bool isTruckOnly(int i);
    double getTime_truck(int i, int j) {
        return instance->time_truck[i][j];
    }

    //    vector<int> getCritical_path(Graph& graph);

    ~Solver();

    bool isFree(int i);
private:
    chrono::time_point<std::chrono::high_resolution_clock> startTime;
    double gap;
    double runtime;
    string status;

    string inputFile;
    string outputFile;
    double time_limit;
    double mem_limit;



    void createModel();
    void createWorkerLP();

    void dispay_solution();
    void write_output();
    string get_details(IloCplex& cplex, NumVar2D& x, IloNumVarArray& z, NumVar2D& f, NumVar2D& y, IloNumVarArray& T);
    vector<int> get_tspTour(IloCplex& cplex, NumVar2D& x);
    vector< vector<int> > get_droneTour(IloCplex& cplex, IloNumVarArray& z, NumVar2D& f, NumVar2D& y);

};

#endif // BENDERSOLVER_H
