#include <iostream>
#include "../lib/solver.hpp"
#include "../lib/hungarian.hpp"
#include <stdio.h>
#include <chrono>
#include <string>
#include <algorithm>
#include <fstream>
#include <sys/time.h>
#include <sys/resource.h>

using namespace std;

int main(int argc, char*argv[]) {
    ifstream infile(argv[3]);
    solver s;
    string results;
    
    string line;
    vector<string> setting;
    bool execute = false;
    while (getline(infile, line))
    {
        string command;
        for (auto word : line) {
            if (execute && !isspace(word)) command += word;
            if (word == '=') execute = true;
        }
        if (execute) setting.push_back(command);
        execute = false;
        line.clear();
    }
    infile.close();

    setpriority(PRIO_PROCESS, 0, -20);
    
    s.assign_parameter(setting);
    results = s.solve(argv[1],atoi(argv[2]));
    
    string instance = argv[1];
    string lib = instance.substr(0,6);
    ofstream resultsFile;
    string file = lib + "Results";
    resultsFile.open(file);
    resultsFile << argv[1] << "," << results << endl;
    resultsFile.close();
    return 0;
}
