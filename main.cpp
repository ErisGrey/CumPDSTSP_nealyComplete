#include <iostream>
#include "config.h"
#include "instance.h"
#include "solver.h"
int main(int argc, char* argv[])
{
    Config config(argc, argv);

    Instance instance(config.input, config.param_input);
    Solver solver(&instance, instance.instanceName, instance.instanceName, config.time_limit, config.mem_limit);
    solver.Solve();

    return 0;
}