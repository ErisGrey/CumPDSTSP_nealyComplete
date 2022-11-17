#include "solver.h"
#include "instance.h"
#include <ilcplex/ilocplex.h>
//#include "utils.h"
//#include "cut.h"
#include <tuple>
ILOSTLBEGIN
#define pos(u,k) min(u,k)][max(u,k)

Solver::Solver(Instance* instance, string input, string output, double time_limit, double mem_limit)
    :instance(instance), inputFile(input), outputFile(output), time_limit(time_limit), mem_limit(mem_limit) {
    cerr << "-- Solving \n";
    startTime = chrono::high_resolution_clock::now();
    outputFile = instance->instanceName;
    gap = 1e5;
    status = "-";

    /*  SET -------------------------------- */
    for (int i = 0; i < instance->num_nodes; ++i)
        N.push_back(i);
    for (int i = 1; i < instance->num_nodes; ++i)
        C.push_back(i);

    // --
    numNode = instance->num_nodes;
    numUAV = instance->num_drones;
    UB = 1e5;
    UB_tsp = 1e5;

    truckOnly = instance->truckonlyCustomer;
    freeCustomers = instance->freeCustomer;
    freeCustomers0 = instance->freeCustomer;
    freeCustomers0.insert(0);
}

Solver::~Solver() {
    //    cerr << "Runtime = " << (double)(clock() - startTime) / CLOCKS_PER_SEC << "s\n";
}

void Solver::Solve() {
    try {
        createModel();

        cplex.exportModel("lpex.lp");
        //        workerCplex.setParam(IloCplex::Param::Preprocessing::Reduce, 0);
        //        workerCplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);
        //        cplex.setParam(IloCplex::Param::Preprocessing::Presolve, IloFalse);
        //        cplex.setParam(IloCplex::Param::MIP::Strategy::Search,
        //                             IloCplex::Traditional);

        cplex.setParam(IloCplex::Param::Parallel, 1);
        cplex.setParam(IloCplex::Param::Threads, 16);
        //        cplex.setParam(IloCplex::Param::Threads, 1);

        //        cplex.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 1e-6);
        cplex.setParam(IloCplex::TiLim, time_limit);
        cplex.setParam(IloCplex::TreLim, mem_limit);

        cplex.setParam(IloCplex::Param::MIP::Strategy::RINSHeur, 10);
        //        cplex.setParam(IloCplex::Param::MIP::Strategy::LBHeur, 1);


                // turn off all auto cuts
        //        cplex.setParam(IloCplex::Param::MIP::Strategy::HeuristicFreq, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::MIRCut, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::Implied, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::Gomory, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::FlowCovers, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::PathCut, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::LiftProj, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::ZeroHalfCut, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::Cliques, -1);
        //        cplex.setParam(IloCplex::Param::MIP::Cuts::Covers, -1);

        //        cplex.use(Lazy(env,*this));
        //        cplex.use(UserTSP(env,*this)); // bug???
        //        cplex.use(UserTSP_old(env,*this)); // bug??? << co ve ngon
        cplex.solve();
        if (cplex.getStatus() == IloAlgorithm::Infeasible) {
            cout << UB << endl;
            cout << "Infeasible" << endl;
        }
        else if (cplex.getStatus() == IloAlgorithm::Unbounded) {
            cout << "Unbounded" << endl;
        }
        else if (cplex.getStatus() == IloAlgorithm::Unknown) {
            cout << "Unknown" << endl;
        }
        else {
            cout << "DONE ..." << endl;
            cout << cplex.getObjValue() << endl;
            dispay_solution();
        }


    }
    catch (IloException& e) {
        cerr << "Conver exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }


    auto endTime = chrono::high_resolution_clock::now();
    runtime = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
    runtime = runtime / 1000;


    write_output();
    cout << "Environment cleared \n";
    //        workerCplex.end();
    cplex.end();

    //        workerEnv.end();
    env.end();
}

void Solver::createModel() {

    model = IloModel(env);
    cplex = IloCplex(env);
    //
    //x = NumVar2D(env, numNode); // x_ij
    //    y = NumVar2D(env, num_nodes); // y_kij
    //    z = NumVar2D(env, numUAV+1); // z_i
    ////    u = IloNumVarArray(env, num_nodes);
    //    v = IloNumVarArray(env, num_nodes);
    //    T = IloNumVarArray (env, num_nodes);
    //    f = NumVar2D(env, num_nodes);
    //    Z = IloNumVar (env, 0, IloInfinity, IloNumVar::Float, "Z");
    //    alpha = IloNumVar (env, 0, IloInfinity, IloNumVar::Float, "alpha");
    //
    //stringstream name;
    //// x_ij
    //for (int i = 0; i < numNode; ++i) {
    //    x[i] = IloNumVarArray(env, numNode);
    //    for (int j = i + 1; j < numNode; ++j) {
    //        if (i == j) continue;
    //        name << "x." << i << "." << j;
    //        x[pos(i, j)] = IloNumVar(env, 0, 1, ILOINT, name.str().c_str());
    //        name.str("");
    //    }
        //    }
        //    // z_i
        //    for (int k = 1; k <= instance->num_drones; ++k){
        //        z[k] = IloNumVarArray(env, num_nodes);
        ////        for (auto j : freeCustomers){
        ////            name << "z." << k << "." << j;
        ////            z[k][j] = IloNumVar(env, 0, 1, ILOINT, name.str().c_str());
        ////            name.str("");
        ////        }
        //    }
        //    for (auto j : freeCustomers){
        //        for (auto dc : instance->c_drone_considered[j]){
        //            name << "z." << dc << "." << j;
        //            z[dc][j] = IloNumVar(env, 0, 1, ILOINT, name.str().c_str());
        //            name.str("");
        //        }
        //    }
        //    // y_ij
        //    for (auto i : freeCustomers0){
        //        y[i] = IloNumVarArray(env, num_nodes);
        //        for (auto j : freeCustomers0){
        //            if (i == j) continue;
        //            name << "y." << i << "." << j;
        //            y[i][j] = IloNumVar(env, 0, 1, ILOINT, name.str().c_str());
        //            name.str("");
        //        }
        //    }
        //    // u_i
        ////    for (int i = 1; i < num_nodes; ++i){
        ////        name << "u." << i;
        ////        u[i] = IloNumVar(env, 0, 1,
        ////                         ILOINT, name.str().c_str());
        ////        name.str("");
        ////    }
        //    // v_i
        //    for (int i = 0; i < num_nodes; ++i){
        //        name << "v." << i;
        //        v[i] = IloNumVar(env, 0, 1, ILOINT, name.str().c_str());
        //        name.str("");
        //    }
        //    // Ti
        //    for (int i : freeCustomers0){
        //        name << "T." << i;
        //        T[i] = IloNumVar(env, 0, IloInfinity,
        //                         IloNumVar::Float, name.str().c_str());
        //        name.str("");
        //    }
        //    for (auto i : freeCustomers0){
        //        f[i] = IloNumVarArray(env, num_nodes);
        //        for (auto j : freeCustomers0){
        //            if (i == j) continue;
        //            name << "f." << i << "." << j;
        //            f[i][j] = IloNumVar(env, 0, numUAV, IloNumVar::Float, name.str().c_str());
        //            name.str("");
        //        }
        //    }
    IloArray<IloNumVarArray> x(env);
    for (int i = 0; i < numNode; i++)
    {
        x.add(IloNumVarArray(env));
        for (int j = 0; j < numNode; j++)
        {
            x[i].add(IloNumVar(env, 0, 1, ILOINT));
        }
    }


    IloArray<IloNumVarArray> y(env);
    for (int i = 0; i < numNode; i++)
    {
        y.add(IloNumVarArray(env));
        for (int j = 0; j < numNode; j++)
        {
            y[i].add(IloNumVar(env, 0, 1, ILOINT));
        }
    }


    IloArray<IloNumVarArray> z(env);
    for (int i = 0; i < numNode; i++)
    {
        z.add(IloNumVarArray(env));
        for (int j = 0; j < numNode; j++)
        {
            z[i].add(IloNumVar(env, 0.0, IloInfinity, ILOFLOAT));
        }
    }


    IloArray<IloNumVarArray> v(env);
    for (int i = 0; i < numNode; i++)
    {
        v.add(IloNumVarArray(env));
        for (int j = 0; j < numNode; j++)
        {
            v[i].add(IloNumVar(env, 0.0, IloInfinity, ILOFLOAT));
        }
    }

    IloNumVarArray u(env, numNode, 1, numNode - 1, ILOFLOAT);
    // OBJ FUNCTION

 //       model.add(IloMinimize(env, Z));
    {
        IloExpr exprSolution(env);
        for (int i = 0; i < numNode; i++)
        {
            for (int j = 0; j < numNode; j++)
            {
                exprSolution += v[i][j] + z[i][j];
            }
        }
        model.add(IloMinimize(env, exprSolution));
    }
    // CONSTRAINT -------------------------------------
    // only one truck out of depot
//    {
//        IloExpr sum(env);
//        for (auto j : C)
//            sum += x[pos(0,j)];
//        model.add( sum == 2 ); // can't fix this
//        sum.end();
//    }
//    // 1. either truck or drone
//    for (auto j : freeCustomers){
//        IloExpr sumz(env);
//        for (int dc : instance->c_drone_considered[j]){
////        for (int dc = instance->min_drones_required[j]; dc <= instance->max_drones_allowed[j]; ++dc){
//            assert(dc != -1);
//            sumz += z[dc][j];
//        }
//
//        model.add( v[j] + sumz == 1 );
//        sumz.end();
//    }
//
//    for (auto j : truckOnly)
//        model.add( v[j] == 1 );
//
//    model.add( v[0] == 1 );
//
//    // linking v and x
//    {
//        for (int i : C){
//            IloExpr sum(env);
//            for (int j : N){
//                if (i == j) continue;
//                sum += x[pos(i,j)];
//            }
//            model.add( sum == 2 * v[i] );
//            sum.end();
//        }
//    }
//
//
//// DRONE ----------------------------------------------------
//    // sum f0i <= numUAV
//    {
//        IloExpr sum(env);
//        for (int i : freeCustomers)
//            sum += f[0][i];
//        model.add( sum == numUAV );
//        sum.end();
//    }
//    // sum_i f_ij == sum_dc dc * z_k_j
//    // drone flow satisfies weight requirement
//    for (auto j : freeCustomers){
//        IloExpr sumF(env);
//        for (auto i : freeCustomers0){
//            if (i == j) continue;
//            sumF += f[i][j];
//        }
//        IloExpr sumZ(env);
//        for (int dc : instance->c_drone_considered[j])
////        for (int dc = instance->min_drones_required[j]; dc <= instance->max_drones_allowed[j]; ++dc)
//            sumZ += dc * z[dc][j];
//
//        model.add( sumF == sumZ );
//        sumF.end();
//        sumZ.end();
//    }
//    // F in = F out
//    for (int j : freeCustomers0){
//        IloExpr sumF1(env);
//        IloExpr sumF2(env);
//        for (int i : freeCustomers0){
//            if (i == j) continue;
//            sumF1 += f[i][j];
//        }
//        for (int i : freeCustomers0){
//            if (i == j) continue;
//            sumF2 += f[j][i];
//        }
//        model.add( sumF1 == sumF2 );
//        sumF1.end();
//        sumF2.end();
//    }
//    // y_ij UPPER BOUND through f_ij
//    for (auto i : freeCustomers0)
//        for (auto j : freeCustomers0){
//            if (i==j) continue;
//            model.add(y[i][j] <= f[i][j]);
//        }
//    // y_ij LOWER BOUND
//    for (auto i : freeCustomers0)
//        for (auto j : freeCustomers0){
//            if (i==j) continue;
//            model.add(y[i][j] >= f[i][j] / instance->num_drones); // y_ij equal 1 as soon ass f_ij > 0
//        }
//
//    // time and drone flow sync
////    model.add(T[0] == 0);
//
//    double bigM = 500;
//    for (auto i: freeCustomers0){
//        for (auto j: freeCustomers){
//            if (i == j) continue;
//            if (i == 0){
//                model.add( T[j] + bigM*(1 - y[i][j]) >= 0 );
//                continue;
//            }
//
//            IloExpr sumT(env);
//            for (int dc : instance->c_drone_considered[i]){
////            for (int dc = instance->min_drones_required[i]; dc <= instance->max_drones_allowed[i]; ++dc){
//                assert(dc != -1);
//                sumT += z[dc][i] * instance->serviceTime_drone(dc, i);
//            }
//
//            model.add( T[j] + bigM*(1 - y[i][j]) >= T[i] + sumT );
//            sumT.end();
//        }
//    }
//
//    // zj y
//    for (auto j : freeCustomers){
//        IloExpr sumy(env);
//        for (auto i : freeCustomers0)
//            if (i != j)
//                sumy += y[i][j];
//
//        IloExpr sum1(env);
//        for (int dc : instance->c_drone_considered[j])
////        for (int dc = instance->min_drones_required[j]; dc <= instance->max_drones_allowed[j]; ++dc)
//            sum1 += z[dc][j] * dc;
//
//        model.add(sum1 >= sumy);
//
//        IloExpr sum2(env);
//        for (int dc : instance->c_drone_considered[j])
////        for (int dc = instance->min_drones_required[j]; dc <= instance->max_drones_allowed[j]; ++dc)
//            sum2 += z[dc][j];
//        model.add(sum2 <= sumy);
//    }
    IloExpr exprCondition1(env);
    for (int i = 1; i < numNode; i++)
    {
        exprCondition1 += x[0][i];
    }
    model.add(exprCondition1 <= 1);

    IloExpr exprCondition2(env);
    for (int i = 1; i < numNode; i++)
    {
        exprCondition2 += y[0][i];
    }
    model.add(exprCondition2 <= numUAV);


    for (int j = 0; j < numNode; j++)
    {
        IloExpr exprCondition3(env);
        for (int i = 0; i < numNode; i++)
        {
            if (i != j)
            {
                exprCondition3 += x[i][j] - x[j][i];
            }
        }
        model.add(exprCondition3 == 0);
    }

    for (int j = 0; j < numNode; j++)
    {
        IloExpr exprCondition4(env);
        for (int i = 0; i < numNode; i++)
        {
            if (i != j)
            {
                exprCondition4 += x[i][j] + y[i][j];
            }
        }
        model.add(exprCondition4 == 1);
    }

    for (int i = 1; i < numNode; i++)
    {
        for (int j = 1; j < numNode; j++)
        {
            model.add(u[i] - u[j] + numNode * x[i][j] <= numNode - 1);
        }
    }

    for (int i = 1; i < numNode; i++)
    {
        IloExpr exprCondition5(env);
        for (int j = 0; j < numNode; j++)
        {
            if (j != i)
            {
                exprCondition5 += v[j][i] + x[i][j] * instance->time_truck[i][j] - v[i][j];
            }
        }
        model.add(exprCondition5 == 0);

        model.add(v[0][i] - instance->time_truck[0][i] * x[0][i] == 0);
    }

    for (int i = 1; i < numNode; i++)
    {
        IloExpr exprCondition6(env);
        for (int j = 0; j < numNode; j++)
        {
            if (j != i)
            {
                exprCondition6 += z[j][i] + y[i][j] * instance->serviceTime_drone(1, j) - z[i][j];
            }
        }
        model.add(exprCondition6 == 0);

        model.add(z[0][i] - instance->serviceTime_drone(1, i) * y[0][i] == 0);
    }
    int BIGM = 500;
    for (int i = 0; i < numNode; i++)
    {
        for (int j = 0; j < numNode; j++)
        {
            if (i != j)
            {
                model.add(v[i][j] >= instance->time_truck[i][j] * x[i][j]);
                model.add(v[i][j] <= BIGM * x[i][j]);
                model.add(z[i][j] >= instance->serviceTime_drone(1, j) * y[i][j]);
                model.add(z[i][j] <= BIGM * y[i][j]);
            }
        }
    }
    //    // no subtour 2 nodes
    //    for (auto i : freeCustomers){
    //        if (instance->min_drones_required[i] == -1) continue;
    //        for (auto j : freeCustomers){
    //            if (i >= j || instance->min_drones_required[j] == -1) continue;
    //            model.add(y[i][j] + y[j][i] <= 1);
    //        }
    //    }



    //    // test -----------------------
    //    model.add(z[1][9] == 1);
    //    model.add(z[1][3] == 1);
    //    model.add(v[7] == 0);
    //    model.add(v[8] == 0);
    //    model.add(v[2] == 0);
    //    model.add(v[1] == 0);
    //    model.add(v[4] == 0);
    //    model.add(v[5] == 0);

    //    model.add(v[11] == 1);
    //    model.add(v[12] == 1);
    //    model.add(v[13] == 1);
    //    model.add(v[14] == 1);
    //    model.add(v[15] == 1);

    //    model.add(y[0][8] == 1);
    //    model.add(y[8][9] == 1);
    //    model.add(y[9][3] == 1);
        // NEW---------------------------------------------
        //    {
        //        IloExpr sum(env);
        //        for (auto i : C)
        //            sum += y[i][0];
        //        model.add( sum <= instance->num_drones );
        //    }

        // truck only -> sum z == 0
    //    {
    //        for (int i : C){
    //            if (isTruckOnly(i)){
    //                IloExpr sum(env);
    //                for (int dc = 1; dc <= numUAV; ++dc)
    //                    sum += z[dc][i];
    //                model.add( sum == 0 );
    //                sum.end();
    //            }
    //        }
    //    }
        // if cant serve -> zki == 0
    //    {
    //        for (int i : C)
    //            for (int dc = 1; dc <= numUAV; ++dc)
    //                if (!servable(dc,i))
    //                    model.add( z[dc][i] == 0 );
    //    }
    //    // a drone only starts 1 trip out from depot -> sum_i y_ij >= u_j
    //    {
    //        for (auto j : C){
    //            IloExpr sumY(env);
    //            for (auto i : N){
    //                if ( i == j ) continue;
    //                sumY += y[i][j];
    //            }
    //            model.add(sumY >= u[j]);
    //            sumY.end();
    //        }
    //    }
        //
    //    {
    //        for (auto j : C){
    //            IloExpr sumY(env);
    //            for (auto i : N){
    //                if ( i == j ) continue;
    //                sumY += y[i][j];
    //            }
    //            IloExpr sumZ(env);
    //            for (int dc = 1; dc <= numUAV; ++dc)
    //                sumZ += z[dc][j] * dc;

    //            model.add(sumY <= sumZ);
    //            sumY.end(); sumZ.end();
    //        }
    //    }

    //    {
    //        IloExpr sum1(env);
    //        IloExpr sum2(env);
    //        for (auto j : C){
    //            sum1 += y[0][j];
    //            sum2 += y[j][0];
    //        }
    //        model.add(sum1 >= 1);
    //        model.add(sum2 >= 1);
    //        sum1.end(); sum2.end();
    //    }


    //    for (int j : C){
    //        IloExpr sum(env);
    //        for (int i : N){
    //            if (i == j) continue;
    //            if (isTruckOnly(i)) continue;
    //            sum += f[0][j];
    //        }
    //        IloExpr sumDrone(env);
    //        for (int dc = 1; dc <= numUAV; ++dc)
    //            sumDrone += z[dc][j];
    //        model.add( sum == sumDrone );
    //    }


    //    for (int i : N){
    //        for (int j : N){
    //            if (i == j) continue;
    //            model.add( f[i][j]*1.0/numUAV <= y[i][j] );
    //            model.add( y[i][j] <= f[i][j] );
    //        }
    //    }
    cplex.extract(model); // <<<< IMPORTANT
    cout << "Done create init MasterProblem\n";

}

//void Solver::updateSubtour_tsp(unordered_map<int,int>& xSol)
//{
//    stour_tsp.clear();
//
//    for (auto& p : xSol){
//
//    }
////    int start = 0;
////    int current = start;
////    vector<int> subtour = {start};
////    while (!xSol.empty()){
////        assert(xSol.find(current) != xSol.end());
////        int temp = xSol[current];
////        xSol.erase(current);
////        current = temp;
////        subtour.push_back(current);
////        if (current == start){
////            assert(subtour.size() > 2);
////            stour_tsp.push_back(subtour);
////            // get start
////            if (!xSol.empty()){
////                start = xSol.begin()->first;
////                current = start;
////                subtour = {start};
////            }
////        }
////    }
//}

/*void Solver::updateSubtour_tsp_lp(vector<Edge>& xSol)
{
    stour_tsp.clear();
    int start = 0;
    int current = start;
    vector<int> subtour = {start};
//    while (!xSol.empty()){
//        for (int i = 0; i < xSol.size(); ++i){
////            if (xSol[i].i == current)
//        }
//        int temp = xSol[current];
//        xSol.erase(current);
//        current = temp;
//        subtour.push_back(current);
//        if (current == start){
//            assert(subtour.size() > 2);
//            stour_tsp.push_back(subtour);
//            // get start
//            if (!xSol.empty()){
//                start = xSol.begin()->first;
//                current = start;
//                subtour = {start};
//            }
//        }
//    }
}*/

//void Solver::updateSubtour_drone(unordered_multimap<int,int> ySol)
//{
//    stour_drone.clear();
//    int start = 0;
//    int current = start;
//    vector<int> subtour = {start};
//    while (!ySol.empty()){
////        assert(ySol.find(current) != ySol.end());
////        int temp = ySol[current];
////        ySol.erase(current);
////        current = temp;
////        subtour.push_back(current);
////        if (current == start){
////            assert(subtour.size() > 2);
////            stour_drone.push_back(subtour);
////            // get start
////            if (!ySol.empty()){
////                start = ySol.begin()->first;
////                current = start;
////                subtour = {start};
////            }
////        }
//    }
//}

//bool Solver::isTruckOnly(int i)
//{
//    return truckOnly.find(i) != truckOnly.end();
//}
//
//bool Solver::isFree(int i)
//{
//    return truckOnly.find(i) == truckOnly.end();
//}


//double Solver::servable(int dc, int customer)
//{
//    if (instance->droneConfigs[dc].flightTime[customer] < 0){
//        return false;
//    } else {
//        return true;
//    }
//}

void Solver::dispay_solution()
{
    double ttruck = 0;
    cout << "V value:\n  ";

    IloArray<IloNumArray> vals(env);
    for (int i = 0; i < numNode; i++)
    {
        for (int j = 0; j < numNode; j++)
        {

            cplex.out() << "z[" << i << "][" << j << "] = " << cplex.getValue(z[i][j]) << endl << endl;

        }
    }
    //cout << "Z value:\n  ";
    //for (auto j : freeCustomers)
    //    for (int dc : instance->c_drone_considered[j]){
    //        if ( cplex.getValue(z[dc][j]) >= 1 - epsilon )
    //             cout << dc << " " << j << ",";
    //    }
    //cout << endl;

//    for (auto i : freeCustomers0)
//        for (auto j : freeCustomers0){
//            if (i == j) continue;
//            if ( cplex.getValue(f[i][j]) >= 1 - epsilon ){
//                cout << "f" << "." << i << "." << j << " = " << cplex.getValue(f[i][j]) << "\n";
//            }
//        }

   /* cout << "Y value:\n  ";
    for (auto i : freeCustomers0){
        for (auto j : freeCustomers0){
            if (i == j) continue;
            if (cplex.getValue(y[i][j]) > 0 && j != 0){
                cout << i << " " << j << " " << cplex.getValue(f[i][j]) << ",";
            }
        }
    }
    cout << endl;*/


    cout << cplex.getObjValue() << endl;
    //    cout << "alpha = " << cplex.getValue(alpha) << "\n";
    //    cout << UB << endl;


}

void Solver::write_output()
{
    //    std::ofstream obest;
    //    obest.open("CPLEX_output_summary.csv", std::ofstream::out | std::ofstream::trunc);
    //    obest.close();
    std::ofstream ocsv;
    ocsv.open("output.txt", std::ios_base::app);
    ocsv << instance->instanceName << ","
        << cplex.getObjValue() << ","
        << cplex.getBestObjValue() << ","
        << fixed << std::setprecision(2)
        << cplex.getMIPRelativeGap() * 100 << ","
        << cplex.getStatus() << ","
        << runtime << "\n"
        << std::flush;
    ocsv.close();
}
