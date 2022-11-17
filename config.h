#pragma once
#ifndef CONFIG_H
#define CONFIG_H
#include <iostream>
#include <string>
#include <chrono>
#include <random>
using namespace std;

class Config
{
public:

    string input;
    string param_input; //C:/Users/admin/Downloads/superDrone/other-params.txt
    int max_iter = 10000;
    double cooling_rate = 0.9995;
    double threshold = 0.15;
    double max_removal_rate = 0.3;
    int seed;

    string output = "output.txt";
    bool found_output = false;

    double time_limit = 10.0;
    bool found_time_limit = false;

    double mem_limit = 28000;
    bool found_mem_limit = false;

    Config(int argc, char* argv[]) {
        for (int i = 1; i < argc; ++i) {
            string key = argv[i];

            if (key == "-i") {
                string value = argv[++i];
                input = value;
                found_input = true;
            }
            if (key == "-p") {
                string value = argv[++i];
                param_input = value;
                found_param_input = true;
            }


        }

        if (!found_input) {
            cerr << "ERROR: Input is required !!!\n";
            exit(0);
        }
        if (!found_output) {
            //	    cout << "Warning: Output is missing !!!!\n";
        }
        if (!found_time_limit) {
            //	    cout << "Warning: time_limit default = 30.0s\n";
        }
        if (!found_max_iter) {
            //            cout << "Warning: max_iter default = 50,000 iterations\n";
        }
        if (!found_cooling_rate) {
            //            cout << "Warning: cooling_rate default = 0.9995\n";
        }
        if (found_seed) {
            //            cout << "Fixed seed = " << seed << "\n";
        }
        else {
            seed = chrono::high_resolution_clock::now().time_since_epoch().count();
            //            cout << "Random seed = " << seed << "\n";
        }
        //        cout << flush;
    }
private:
    bool found_input = false;
    bool found_param_input = false;
    bool found_max_iter = false;
    bool found_cooling_rate = false;
    bool found_threshold = false;
    bool found_max_removal_rate = false;
    bool found_seed = false;
};

#endif // CONFIG_H
#pragma once
