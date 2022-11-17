#include "instance.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <map>
#include <string>
#include <iostream>
#include <ilcplex/ilocplex.h>

Instance::Instance(string instanceFile, string paramsFile)
{

    read_input(instanceFile);
    read_otherParams(paramsFile);
    initialize();

    // ENERGY MODEL
    energyModel = new EnergyModel(h_flight, v_takeoff, v_landing, air_density_rho, peukert_constant);
    //    energyModel.h_flight = h_flight;
    //    energyModel.v_takeoff = v_takeoff;
    //    energyModel.v_landing = v_landing;
    //    energyModel.rho = air_density_rho;

        // gen all kind of possible collective drone configs

    for (int i = 0; i < getNum_drones() + 1; ++i) {
        droneConfigs.emplace_back(i,
            frame_weight,
            battery_weight,
            num_propellers,
            rotor_blade_radius,
            battery_capacity,
            voltage,
            max_discharge_current
        );
        droneConfigs[i].flightTime.resize(num_nodes, -1);
    }
    cout.precision(10);
    //   int upper = min(num_drones, max_combination);
    for (int customer = 1; customer < getNum_nodes(); ++customer) {
        //        cout << "CUSTOMER " << customer << ": " << getWeight(customer) << "\n    ";
        double dist = dist_drone[0][customer] * 1000; // km to m
        //        cout << customer << " " << dist << endl;
        cout << droneConfigs[1].frame_weight << endl;
        cout << droneConfigs[1].battery_weight << endl;

        if (getWeight(customer) + droneConfigs[1].getW_single_drone() > max_weight_allowed)
            continue;

        DroneConfig& dc = droneConfigs[1];
        vector<double> optimal_setting = energyModel->get_optimal_power(dc, getWeight(customer), dist);

        cout << "optimal_setting[0] = " << optimal_setting[0] << endl;

        if (optimal_setting[0] != -1) {
            dc.flightTime[customer] =
                dist / optimal_setting[0] + // inbound trip in second
                dist / optimal_setting[1] + // outbound trip in second
                2 * energyModel->h_flight / energyModel->v_takeoff + // 2 times takeoff in second
                2 * energyModel->h_flight / energyModel->v_landing  // 2 time landing in second
                ;
            //                dc.flightTime[customer] = Utils::round( dc.flightTime[customer]/60 , 0 ); // /60 to minute
            //                cout << optimal_setting[0] << "-" << optimal_setting[1] << " ";
        }
        else {
            //                cout  << "X ";
        }

        //        cout << endl;
    }
    for (int i = 1; i < num_nodes; i++)
    {
        cout << "tdrone[" << i << "] =" << serviceTime_drone(1, i) << endl;
    }

    //    energyModel->show_discharge_curve(droneConfigs[1]);


        /*min_drones_required.resize(num_nodes, -1);
        max_drones_allowed.resize(num_nodes, -1);
        for (int i = 1; i < num_nodes; ++i)
            for (int dc = 1; dc <= upper; ++dc){
                if (tdrone(dc, i) < 0)
                    continue;
                if (getWeight(i) + droneConfigs[dc].getW_single_drone()*dc > max_weight_allowed)
                    continue;
                if (min_drones_required[i] == -1)
                    min_drones_required[i] = dc;
                max_drones_allowed[i] = dc;
            }
        for (int i = 1; i < num_nodes; ++i){
            if (isTruckonly(i))
                truckonlyCustomer.insert(i);
            else
                freeCustomer.insert(i);
        }

        for (int i : freeCustomer){
            c_drone_considered[i].push_back( min_drones_required[i] );
            double bestTime = serviceTime_drone(min_drones_required[i],i);
            for (int dc = min_drones_required[i]; dc <= max_drones_allowed[i]; ++dc){
                if (serviceTime_drone(dc,i) < bestTime){
                    c_drone_considered[i].push_back(dc);
                    bestTime = serviceTime_drone(dc,i);
                }
            }
        }*/

    return;
}

Instance::~Instance()
{
    delete energyModel;
}

void Instance::read_input(const string& inputFile)
{
    ifstream myFile(inputFile);

    if (!myFile.is_open())
    {
        // End program immediately
        cout << "Unable to open instance file \n";
        exit(0);
    }

    string str;
    while (!myFile.eof()) {
        string ss;
        getline(myFile, ss);
        ss += " ";
        str += ss;
    }
    stringstream ss(str);
    string Token;
    while (ss >> Token) {

        if (Token == "weight")
        {
            break;
        }

    }
    vector<float> Data;
    while (ss >> Token) {
        float f;
        f = atof(Token.c_str());
        Data.push_back(f);
    }
    //  So node
    num_nodes = Data.size() / 4;
    cout << "Dimension: " << num_nodes << endl;
    for (int i = 0; i < num_nodes; i++)
    {
        x.push_back(Data.at(4 * i + 1));
        y.push_back(Data.at(4 * i + 2));
        weight.push_back(Data.at(4 * i + 3));
        /*      cout << "x[" << i << "]=" << x[i] << endl;
              cout << "y[" << i << "]=" << y[i] << endl;
              cout << "weight[" << i << "]=" << weight[i] << endl;*/
    }


    ifstream myFile1(inputFile);
    string line;
    vector< string > numbers;

    getline(myFile1, line);
    split(line, numbers, ',');
    num_drones = stoi(numbers[1]);

    getline(myFile1, line);
    split(line, numbers, ',');
    v_truck = stod(numbers[1]);

    getline(myFile1, line);
    split(line, numbers, ',');
    battery_weight = stod(numbers[1]);

    getline(myFile1, line);
    split(line, numbers, ',');
    battery_capacity = stod(numbers[1]);

    getline(myFile1, line);
    split(line, numbers, ',');
    max_discharge_current = stod(numbers[1]);

    getline(myFile1, line);
    split(line, numbers, ',');
    voltage = stod(numbers[1]);


    //    getline(myFile, line);
    //    split(line, numbers, ',');
    //    endurance_drone = stoi(numbers[1]);
    //
    //   getline(myFile1, line);
    //    num_nodes = 0;
    //    while (getline(myFile1, line))
    //    {
    //        cout << "16" << endl;
    //        split(line, numbers, '\t');
    //        //x.push_back( stoi(numbers[1]) );
    //        //y.push_back( stoi(numbers[2]) );
    //        float w = stod(numbers[3]);
    //        weight.push_back( w ); 
    ////        if (w > cap_drone){
    ////            overweight.push_back(1);
    ////        } else {
    ////            overweight.push_back(0);
    ////        }
    //        num_nodes++;
    //    } 
    //    weight[0] = 0;
    //    for (int i = 0; i < num_nodes; ++i){
    //        weight[i] = std::floor(weight[i] * 1000) / 1000; //Utils::round(weight[i], 3);
    //        //weight[i] = Utils::round(weight[i], 2);
    //        cout << "weight[" << i << "] = " << weight[i] << endl;
    //    } 
    myFile1.close();
}

void Instance::read_otherParams(const string& paramsFile)
{
    ifstream myFile(paramsFile);
    if (!myFile.is_open()) {
        cout << "Unable to open params file \n";
        exit(0);
    }

    string line;
    string token;
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        coupletime = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        max_combination = stoi(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        max_weight_allowed = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        frame_weight = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        num_propellers = stoi(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        rotor_blade_radius = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        peukert_constant = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        h_flight = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        v_takeoff = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        v_landing = stod(token);
    }
    getline(myFile, line);
    {
        stringstream ss(line);
        getline(ss, token, ',');
        getline(ss, token, ',');
        air_density_rho = stod(token);
    }

    //    myFile >> token >> drone_prepare_time;
    //    myFile >> token >> frame_weight;
    //    myFile >> token >> num_propellers;
    //    myFile >> token >> rotor_blade_radius;
    //    myFile >> token >> peukert_constant;
    //    myFile >> token >> h_flight;
    //    myFile >> token >> v_takeoff;
    //    myFile >> token >> v_landing;
    //    myFile >> token >> air_density_rho;
}

void Instance::initialize()
{
    // Compute distance and time matrix
    time_truck.resize(num_nodes);
    dist_drone.resize(num_nodes);
    dist_truck.resize(num_nodes);
    for (int i = 0; i < num_nodes; i++)
    {
        time_truck[i].resize(num_nodes);
        dist_drone[i].resize(num_nodes);
        dist_truck[i].resize(num_nodes);
    }
    for (int i = 0; i < num_nodes; i++) {
        for (int j = 0; j < num_nodes; j++) {
            float euc_d = pow(pow(x[i] - x[j], 2) + pow(y[i] - y[j], 2), 0.5);
            float man_d = abs(x[i] - x[j]) + abs(y[i] - y[j]);

            dist_drone[i][j] = euc_d;
            dist_truck[i][j] = man_d;

            //            time_drone[i][j] = round(euc_d/v_drone);
            time_truck[i][j] = man_d / v_truck * 60.0;
            //            time_truck[i][j] = Utils::round( time_truck[i][j] , 0 );
        }
    }



    //    ctype.resize(num_nodes, 0);
    //    for (int i = 1; i < num_nodes; ++i){
    //        if ( tdrone(i) >  endurance_drone ||
    //             get_num_drones_needed(i) > num_drones ){
    //            ctype[i] = 0;
    //            truckonly.push_back(i);
    //        }
    //        else{
    //            ctype[i] = 1;
    //            freemode.push_back(i);
    //        }
    //    }

        // polar
    polar.push_back(0);
    for (int i = 1; i < num_nodes; ++i) {
        double dx = x[i] - x[0];
        double dy = y[i] - y[0];
        double a = atan(dy / dx) * 180.0 / 3.14159265;
        if (dx >= 0 && dy >= 0) {

        }
        else if (dx < 0 && dy >= 0) {
            a += 180;
        }
        else if (dx < 0 && dy < 0) {
            a += 180;
        }
        else if (dx >= 0 && dy < 0) {
            a += 360;
        }

        polar.push_back(a);
        //        cout << i <<" "<< a << "\n";
    }

    //int n = num_nodes;
    //adjList.resize(num_nodes);
    //vector<int> customers;
    //for (int i = 0; i < n; i++){
    //    if (i!=0)
    //        customers.push_back(i);
    //    adjList[i].reserve(n);
    //}

    //for (int node = 1; node < n; ++node)
    //{
    //    // sort dec
    //    sort(customers.begin(), customers.end(),
    //         [this,node](const int &a, const int & b) -> bool
    //    {
    //        return dist_drone[a][node] < dist_drone[b][node];
    //    });
    //    adjList[node] = customers;
    //} 
}

double Instance::tdrone(const int& dconfig, const int& customer) const
{
    return droneConfigs[dconfig].flightTime[customer];
}

double Instance::serviceTime_drone(const int& dconfig, const int& customer) const
{
    if (dconfig > 1) {
        return tdrone(dconfig, customer) + coupletime;
    }
    else {
        return tdrone(dconfig, customer);
    }
}

double Instance::ttruck(int i, int j)
{
    return time_truck[i][j];
}

double Instance::dtruck(int i, int j)
{
    return dist_truck[i][j];
}

int Instance::getNum_nodes() const
{
    return num_nodes;
}

int Instance::getNum_drones() const
{
    return num_drones;
}

double Instance::getWeight(int i) const
{
    return weight[i];
}

bool Instance::isTruckonly(int customer)
{
    if (min_drones_required[customer] == -1 || min_drones_required[customer] > max_combination || getWeight(customer) > max_weight_allowed)
        return true;
    else
        return false;
}

void Instance::export_stat()
{
    // count inreach, overweight, drone-el
    int inreach = 0;
    int overw = 0;
    int overw_inreach = 0;
    int el = 0;
    //    for (int i = 1; i < num_nodes; ++i){
    //        if (tdrone(i)/v_drone <= endurance_drone)
    //            ++inreach;

    //        if (weight[i] > cap_drone)
    //            ++overw;

    //        if (tdrone(i)/v_drone <= endurance_drone && weight[i] > cap_drone && ceil(weight[i] / cap_drone) <= num_drones)
    //            ++overw_inreach;

    //        if (tdrone(i)/v_drone <= endurance_drone && ceil(weight[i] / cap_drone) <= num_drones)
    //            ++el;
    //    }

    string s = "";
    s += instanceName + ",";
    s += to_string(num_nodes - 1) + ",";
    s += to_string(inreach) + ",";
    s += to_string(overw) + ",";
    s += to_string(overw_inreach) + ",";
    s += to_string(el);

    std::ofstream outCSV;
    outCSV.open("instance_stat_summary.csv", std::ios_base::app);
    outCSV << s << "\n" << std::flush;
    outCSV.close();
}

template<class Container>
void Instance::split(const std::string& str, Container& cont, char delim)
{
    cont.clear();
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        if (token != "")
            cont.push_back(token);
    }
}
