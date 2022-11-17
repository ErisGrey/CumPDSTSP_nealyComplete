

#include <ilcplex/ilocplex.h>
ILOSTLBEGIN

int
main()
{
    IloEnv   env;
    try {

        IloModel model(env, "chvatal");

        IloCplex cplex(model);
        cplex.setParam(IloCplex::TiLim, 1800);
        IloNum start = cplex.getTime();
        char f[99999];
        float n[99999];
        int i = 0;
        ifstream infile("C:\\Users\\admin\\Downloads\\WEISH02.csv");
        string str;
        while (!infile.eof()) {
            string ss;
            getline(infile, ss);
            ss += " ";
            str += ss;
        }
        stringstream ss(str);

        string token;
        while (ss >> token) {
            n[i] = atof(token.c_str());

            i++;
        }
        int NumKnapsack = n[0];
        int NumObject = n[1];
        float TotalWeight[10000];
        float weight[100][100];
        float value[10000];
        for (int i = 0; i < NumObject; i++) {
            value[i] = n[i + 2];
            cplex.out() << "value[" << i << "] = " << value[i] << endl;
        }
        for (int i = 0; i < NumKnapsack; i++) {
            TotalWeight[i] = n[NumObject + i + 2];
            cplex.out() << "TotalWeight[" << i << "] = " << TotalWeight[i] << endl;
        }
        for (int i = 0; i < NumKnapsack; i++)
        {
            for (int j = 0; j < NumObject; j++) {
                weight[i][j] = n[(i + 1) * NumObject + NumKnapsack + j + 2];
                cplex.out() << "weight[" << i << "][" << j <<"] = " << weight[i][j] << endl;
            }
        }


        IloNumVarArray x(env, NumObject, 0, 1, ILOINT);
        IloExpr exprSolution(env);
        for (size_t i = 0; i < NumObject; i++)
        {
            exprSolution += x[i] * value[i];
        }
        model.add(IloMaximize(env, exprSolution));
        for (int j = 0; j < NumKnapsack; j++)
        {
            IloExpr exprCondition(env);
            for (int i = 0; i < NumObject; i++)
            {
                exprCondition += x[i] * weight[j][i];
            }
            model.add(exprCondition
                <= TotalWeight[j]);
        }
        
        cplex.solve();
        IloNumArray vals(env);
        cplex.getValues(vals, x);
        cplex.out() << "Number Knapsack " << NumKnapsack << endl;
        cplex.out() << "Number Object " << NumObject << endl;
        cplex.out() << "Solution status " << cplex.getStatus() << endl;
        cplex.out() << "Optimal Value " << n[2 + (NumKnapsack + 1) * NumObject + NumKnapsack] << endl;
        cplex.out() << "Objective value " << cplex.getObjValue() << endl;

        cplex.out() << "Time:  " << cplex.getTime() - start << endl; 

        cplex.exportModel("lpex3.lp"); 
    }
    catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }
    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }
    
    env.end(); 
    return 0;
}
