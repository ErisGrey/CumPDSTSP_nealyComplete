
/*
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
        ifstream infile("C:\\Users\\admin\\Downloads\\large_scale\\knapPI_3_10000_1000_1.csv");
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
        int NumArray = n[0];
        float TotalWeight = n[1];
        float weight[10000];
        float price[10000];
        for (int i = 0; i < NumArray; i++) {
           
                    weight[i] = n[2 * i + 3];
                    price[i] = n[2 * i + 2];  
                    
            
           
        }
       
        

        IloNumVarArray x(env, NumArray, 0, 1, ILOINT);
        IloExpr exprSolution(env);
        for (size_t i = 0; i < NumArray; i++)
        {
            exprSolution += x[i] * price[i];
        }
        model.add(IloMaximize(env, exprSolution ));
        IloExpr exprCondition(env);
         for(size_t i = 0; i < NumArray; i++)
        {
            exprCondition += x[i] * weight[i];
        }
        model.add(exprCondition
            <= TotalWeight);
        cplex.solve();
        IloNumArray vals(env);
        cplex.getValues(vals, x);
        cplex.out() << "Total Weight" << n[1] << endl;
        cplex.out() << "Solution status " << cplex.getStatus() << endl;
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
} */
