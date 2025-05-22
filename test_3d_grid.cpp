#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <math.h>
#include <tuple>
#include <algorithm>
#include <filesystem>
#include <sys/stat.h>
#include <unistd.h>
#include <boost/algorithm/string.hpp>
#include "3dgrid.h"
#include "ACLS.h"
#include "LACLS.h"
#include "frontier.h"
#include "astar.h"
#include "smgs.h"
#include "readMap.h"
using namespace std;

bool readStartGoalPairs(const string &filepath, unordered_map<string, vector<vector<int>>> *pairs)
{
    ifstream fin(filepath);
    if (!fin.is_open())
    {
        return false;
    }

    string line;
    string token;
    string delimiter = " ";
    vector<string> tokens;

    int start_row, start_col, start_height, goal_row, goal_col, goal_height;

    // Skip first 2 lines
    getline(fin, line);
    getline(fin, line);
    string map = "./3d_maps/" + line;

    int i = 0;

    while (getline(fin, line))
    {

        boost::split(tokens, line, boost::is_any_of(delimiter));

        start_row = stoi(tokens[0]);
        start_col = stoi(tokens[1]);
        start_height = stoi(tokens[2]);
        goal_row = stoi(tokens[3]);
        goal_col = stoi(tokens[4]);
        goal_height = stoi(tokens[5]);

        if ((*pairs).count(map) == 0)
        {
            (*pairs).insert(make_pair(map, vector<vector<int>>(1, {start_row, start_col, start_height, goal_row, goal_col, goal_height})));
        }
        else
        {
            (*pairs)[map].push_back({start_row, start_col, start_height, goal_row, goal_col, goal_height});
        }
    }
    return true;
}

void test_maps(vector<string> map_fpath, bool run_astar, bool run_frontier, bool run_ACLS, bool run_LACLS, bool run_smgs)
{
    int ***stateMap;
    int row_size, col_size, height_size;

    unordered_map<string, vector<vector<int>>> pairs;

    for (string map : map_fpath)
    {
        readStartGoalPairs(map + ".3dscen", &pairs);

        if (!read3DMap(map, &stateMap, &row_size, &col_size, &height_size))
        {
            std::printf("Failure\n");
        }

        std::vector<tuple<int, int, int>> actions = {make_tuple(0, 0, -1), make_tuple(-1, 0, 0), make_tuple(0, -1, 0), make_tuple(0, 1, 0), make_tuple(1, 0, 0), make_tuple(0, 0, 1)};
        std::vector<double> costs = {1, 1, 1, 1, 1, 1};

        int iters = 1000; // pairs[map].size();

        // Weight for the heuristic
        int weight = 1;

        // Weight for the attractor distance function
        int dist_weight = 1;

        vector<vector<double>> planner_times(iters, vector<double>(5, 0.0));
        vector<vector<int>> solution_lens(iters, vector<int>(5, 0));
        vector<vector<int>> closed_list_size(iters, vector<int>(5, 0));
        for (int i = 0; i < iters; i++)
        {
            tuple<int, int, int> start_state = make_tuple(pairs[map][i][0], pairs[map][i][1], pairs[map][i][2]);
            tuple<int, int, int> goal_state = make_tuple(pairs[map][i][3], pairs[map][i][4], pairs[map][i][5]);

            printf("map: %s, start state: %d,%d,%d  goal state: %d,%d,%d\n", map.c_str(), get<0>(start_state), get<1>(start_state), get<2>(start_state), get<0>(goal_state), get<1>(goal_state), get<2>(goal_state));

            vector<int> solution_ids;
            int cost;
            double time_span = 0;
            double astar_time, frontier_time, ACLS_time, LACLS_time, SMGS_time = 0;
            chrono::high_resolution_clock::time_point t1, t2;

            int start_state_id, goal_state_id;
            if (run_ACLS)
            {
                Grid3DEnvironment GridEnv;
                if (!GridEnv.init(stateMap, row_size, col_size, height_size, actions, costs, "ACLS"))
                {
                    throw runtime_error("Map Initialization failed");
                }

                GridEnv.setStartState(start_state);
                GridEnv.setGoalState(goal_state);
                GridEnv.setWeight(dist_weight);

                start_state_id = GridEnv.getStartStateID();
                goal_state_id = GridEnv.getGoalStateID();

                ACLS ACLSPlanner(&GridEnv);
                ACLSPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();
                ACLSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);

                std::vector<GridState *> solution_states;
                GridEnv.extractPath("attractors", solution_ids, solution_states);

                t2 = chrono::high_resolution_clock::now();
                ACLS_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("ACLS Planner time: %f\n", ACLS_time);
                printf("ACLS Pathlen: %d\n", (int)solution_ids.size());

                planner_times[i][2] = ACLS_time;
                solution_lens[i][2] = (int)solution_ids.size();
                closed_list_size[i][2] = ACLSPlanner.getNumAttractors();

                solution_ids.clear();
                printf("ACLS Num of attractors: %d\n", ACLSPlanner.getNumAttractors());
            }

            if (run_LACLS)
            {
                Grid3DEnvironment GridEnv;
                if (!GridEnv.init(stateMap, row_size, col_size, height_size, actions, costs, "LACLS"))
                {
                    throw runtime_error("Map Initialization failed");
                }

                GridEnv.setStartState(start_state);
                GridEnv.setGoalState(goal_state);
                GridEnv.setWeight(dist_weight);

                start_state_id = GridEnv.getStartStateID();
                goal_state_id = GridEnv.getGoalStateID();

                LACLS LACLSPlanner(&GridEnv);
                LACLSPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();
                int err = LACLSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                t2 = chrono::high_resolution_clock::now();
                LACLS_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

                std::vector<GridState *> solution_states;
                GridEnv.extractPath("attractors", solution_ids, solution_states);

                printf("LACLS Planner time: %f\n", LACLS_time);
                printf("LACLS Num of attractors: %d\n", LACLSPlanner.getNumAttractors());
                printf("LACLS Pathlen: %d\n", (int)solution_ids.size());
                planner_times[i][3] = LACLS_time;
                solution_lens[i][3] = (int)solution_ids.size();
                closed_list_size[i][3] = LACLSPlanner.getNumAttractors();

                solution_ids.clear();
            }

            if (run_frontier)
            {
                Grid3DEnvironment GridEnv;
                if (!GridEnv.init(stateMap, row_size, col_size, height_size, actions, costs, "FA*"))
                {
                    throw runtime_error("Map Initialization failed");
                }

                GridEnv.setStartState(start_state);
                GridEnv.setGoalState(goal_state);
                GridEnv.setWeight(dist_weight);

                start_state_id = GridEnv.getStartStateID();
                goal_state_id = GridEnv.getGoalStateID();
                FrontierAStar FrontierPlanner(&GridEnv);
                FrontierPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();

                FrontierPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);

                std::vector<GridState *> frontier_solution_states;
                GridEnv.extractPath("frontier", solution_ids, frontier_solution_states);

                t2 = chrono::high_resolution_clock::now();
                frontier_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("FA* Planner time: %f\n", frontier_time);
                printf("FA* Pathlen: %d\n", (int)solution_ids.size());

                planner_times[i][1] = frontier_time;
                solution_lens[i][1] = (int)solution_ids.size();
                closed_list_size[i][1] = 0;

                solution_ids.clear();
            }

            if (run_astar)
            {
                Grid3DEnvironment GridEnv;

                if (!GridEnv.init(stateMap, row_size, col_size, height_size, actions, costs, "A*"))
                {
                    throw runtime_error("Map Initialization failed");
                }

                GridEnv.setStartState(start_state);
                GridEnv.setGoalState(goal_state);
                GridEnv.setWeight(dist_weight);

                start_state_id = GridEnv.getStartStateID();
                goal_state_id = GridEnv.getGoalStateID();
                AStar AStarPlanner(&GridEnv);
                AStarPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();
                AStarPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);

                std::vector<GridState *> astar_solution_states;
                GridEnv.extractPath("astar", solution_ids, astar_solution_states);
                t2 = chrono::high_resolution_clock::now();
                astar_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

                printf("A* Planner time: %f\n", astar_time);
                printf("A* Pathlen: %d\n", (int)solution_ids.size());
                printf("A* expansions: %d\n", AStarPlanner.getExpansions());

                planner_times[i][0] = astar_time;
                solution_lens[i][0] = (int)solution_ids.size();
                closed_list_size[i][0] = AStarPlanner.getExpansions();

                solution_ids.clear();
            }

            if (run_smgs)
            {
                Grid3DEnvironment GridEnv;

                if (!GridEnv.init(stateMap, row_size, col_size, height_size, actions, costs, "smgs"))
                {
                    throw runtime_error("Map Initialization failed");
                }
                GridEnv.setStartState(start_state);
                GridEnv.setGoalState(goal_state);
                GridEnv.setWeight(dist_weight);

                start_state_id = GridEnv.getStartStateID();
                goal_state_id = GridEnv.getGoalStateID();
                SMGS SMGSPlanner(&GridEnv);
                SMGSPlanner.setWeight(weight);

                SMGSPlanner.setClosedThreshold(closed_list_size[i][0] * 0.1);
                SMGSPlanner.setClosedMaxThreshold(closed_list_size[i][0] * 0.5); // Set to 0.5*A* closed size
                t1 = chrono::high_resolution_clock::now();
                SMGSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                std::vector<GridState *> SMGS_solution_states;
                GridEnv.extractPath("SMGS", solution_ids, SMGS_solution_states);
                t2 = chrono::high_resolution_clock::now();
                SMGS_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

                planner_times[i][4] = SMGS_time;
                solution_lens[i][4] = (int)solution_ids.size();
                closed_list_size[i][4] = SMGSPlanner.getClosedMaxSize();

                printf("SMGS Planner time: %f\n", SMGS_time);
                printf("SMGS Closed list size: %d\n", SMGSPlanner.getClosedMaxSize());
                printf("Pathlen: %d\n", (int)solution_ids.size());

                solution_ids.clear();
            }
            printf("\n\n");

            // Record the results
            // string map_str = map;
            // boost::erase_all(map_str, "./");
            // boost::erase_all(map_str, "/");
            // boost::erase_all(map_str, ".3dmap");
            // boost::erase_all(map_str, "3d_maps");
            // string file = "planner_times_3D_" + map_str + ".txt";
            // FILE *fout = fopen(file.c_str(), "a");
            // fprintf(fout, "solution_len %d closed: %d %d %d %d time %f %f %f %f\n", solution_lens[i][0], closed_list_size[i][0], closed_list_size[i][1], closed_list_size[i][2], closed_list_size[i][3], planner_times[i][0], planner_times[i][1], planner_times[i][2], planner_times[i][3]);
            // fclose(fout);
        }

        free3DMap(stateMap, row_size, col_size);
    }

    return;
}

int main()
{

    vector<string> maps;
    maps.push_back("./3d_maps/A1.3dmap");
    maps.push_back("./3d_maps/BA1.3dmap");
    maps.push_back("./3d_maps/DC3.3dmap");

    test_maps(maps, true, true, true, true, false);

    return 0;
}

//  g++ -o 3dgrid.out 3dgrid.h 3dgrid.cpp compares.h compares.cpp ACLS.h ACLS.cpp LACLS.h LACLS.cpp frontier.h frontier.cpp astar.h astar.cpp smgs.h smgs.cpp readMap.h readMap.cpp test_3d_grid.cpp
