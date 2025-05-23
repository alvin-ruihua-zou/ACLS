#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <math.h>
#include <algorithm>
#include <unistd.h>
#include <boost/algorithm/string.hpp>
#include "2dgrid.h"
#include "ACLS.h"
#include "LACLS.h"
#include "frontier.h"
#include "astar.h"
#include "smgs.h"
#include "readMap.h"

using namespace std;
using namespace Grid2D;

bool readStartGoalPairs(const string &filepath, unordered_map<string, vector<vector<int>>> *pairs)
{
    ifstream fin(filepath);
    if (!fin.is_open())
    {
        return false;
    }

    string line;
    string map;
    int start_row, start_col, goal_row, goal_col;
    while (getline(fin, line))
    {
        if (line.substr(0, line.find(" ")) == "Map")
        {
            map = line.substr(line.find(" ") + 1);
        }
        else
        {
            size_t last = 0;
            size_t next = 0;
            vector<string> tokens;
            while ((next = line.find(" ", last)) != string::npos)
            {
                tokens.push_back(line.substr(last, next - last));
                last = next + 1;
            }
            tokens.push_back(line.substr(last));

            start_row = stoi(tokens[1]);
            start_col = stoi(tokens[2]);
            goal_row = stoi(tokens[4]);
            goal_col = stoi(tokens[5]);

            if ((*pairs).count(map) == 0)
            {
                (*pairs).insert(make_pair(map, vector<vector<int>>(1, {start_row, start_col, goal_row, goal_col})));
            }
            else
            {
                (*pairs)[map].push_back({start_row, start_col, goal_row, goal_col});
            }
        }
    }
    return true;
}

void test_maps(vector<string> map_fpath, bool run_astar, bool run_frontier, bool run_ACLS, bool run_LACLS, bool run_smgs)
{
    int **stateMap;
    int row_size, col_size;

    unordered_map<string, vector<vector<int>>> pairs;
    readStartGoalPairs("../start_goal_pairs.txt", &pairs);

    printf("Pairs size: %d\n", (int)pairs.size());

    for (string map : map_fpath)
    {
        if (!readMap(map, &stateMap, &row_size, &col_size))
        {
            std::printf("Failure\n");
        }

        GridEnvironment GridEnv;
        std::vector<std::pair<int, int>> actions;
        std::vector<double> costs;
        for (std::pair<int, int> action : {make_pair(-1, 0), make_pair(0, -1), make_pair(0, 1), make_pair(1, 0)})
        {
            actions.push_back(action);
            costs.push_back(1);
        }

        int iters = 500; // pairs[map].size();

        vector<vector<double>> planner_times(iters, vector<double>(5, 0.0));
        vector<vector<int>> solution_lens(iters, vector<int>(5, 0));
        vector<vector<int>> closed_list_size(iters, vector<int>(5, 0));

        vector<int> open_sizes(iters, 0);

        // Heuristic weight
        int weight = 1;

        // Attractor distance function weight
        int dist_weight = 1;

        for (int i = 0; i < iters; i++)
        {
            pair<int, int> start_state = make_pair(pairs[map][i][0], pairs[map][i][1]);
            pair<int, int> goal_state = make_pair(pairs[map][i][2], pairs[map][i][3]);

            printf("map: %s, index: %d, start state: %d,%d  goal state: %d,%d\n", map.c_str(), i, start_state.first, start_state.second, goal_state.first, goal_state.second);
            vector<int> solution_ids;
            int cost;
            double time_span = 0;
            double astar_time, frontier_time, ACLS_time, LACLS_time, SMGS_time = 0;
            chrono::high_resolution_clock::time_point t1, t2;

            int start_state_id, goal_state_id;

            if (run_ACLS)
            {
                if (!GridEnv.init(stateMap, row_size, col_size, actions, costs, "ACLS"))
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
                if (!GridEnv.init(stateMap, row_size, col_size, actions, costs, "LACLS"))
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
                if (err)
                {
                    printf("Planning failed\n");
                }
                std::vector<GridState *> solution_states;
                GridEnv.extractPath("attractors", solution_ids, solution_states);

                t2 = chrono::high_resolution_clock::now();
                LACLS_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("LACLS Planner time: %f\n", LACLS_time);

                planner_times[i][3] = LACLS_time;
                solution_lens[i][3] = (int)solution_ids.size();
                closed_list_size[i][3] = LACLSPlanner.getNumAttractors();

                printf("LACLS Num of attractors: %d\n", LACLSPlanner.getNumAttractors());
                printf("LACLS Pathlen: %d\n", (int)solution_ids.size());
                solution_ids.clear();
            }

            if (run_frontier)
            {
                if (!GridEnv.init(stateMap, row_size, col_size, actions, costs, "FA*"))
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
                if (!GridEnv.init(stateMap, row_size, col_size, actions, costs, "A*"))
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

                planner_times[i][0] = astar_time;
                solution_lens[i][0] = (int)solution_ids.size();
                closed_list_size[i][0] = AStarPlanner.getExpansions();

                open_sizes[i] = AStarPlanner.getOpenSize();

                printf("A* Planner time: %f\n", astar_time);
                printf("A* Pathlen: %d\n", (int)solution_ids.size());
                printf("A* expansions: %d\n", AStarPlanner.getExpansions());

                solution_ids.clear();
            }

            if (run_smgs)
            {
                if (!GridEnv.init(stateMap, row_size, col_size, actions, costs, "smgs"))
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

            // For recording results
            // string map_str = map;
            // boost::erase_all(map_str, "./");
            // boost::erase_all(map_str, "/");
            // boost::erase_all(map_str, ".map");
            // boost::erase_all(map_str, "maps");
            // string file = "planner_times_2D_smgs_" + map_str + "_w=" + to_string(weight) + ".txt";
            // printf("%s\n", file.c_str());
            // FILE *fout = fopen(file.c_str(), "a");
            // fprintf(fout, "solution_len %d closed: %d %d %d %d %d time %f %f %f %f %f\n", solution_lens[i][0], closed_list_size[i][0], closed_list_size[i][1], closed_list_size[i][2], closed_list_size[i][3], closed_list_size[i][4], planner_times[i][0], planner_times[i][1], planner_times[i][2], planner_times[i][3], planner_times[i][4]);
            // fclose(fout);

            // // For recording memory
            // string map_str_mem = map;
            // boost::erase_all(map_str_mem, "./");
            // boost::erase_all(map_str_mem, "/");
            // boost::erase_all(map_str_mem, ".map");
            // boost::erase_all(map_str_mem, "maps");
            // string file_mem = "planner_memory_2D_termination_" + map_str_mem + "_w=" + to_string(weight) + ".txt";
            // printf("%s\n", file_mem.c_str());
            // FILE *fout_mem = fopen(file_mem.c_str(), "a");
            // // fprintf(fout_mem, "solution_len %d closed: %d %d time %f %f\n", solution_lens[i][4], closed_list_size[i][0], closed_list_size[i][4], planner_times[i][0], planner_times[i][4]);
            // fprintf(fout_mem, "solution_len %d open %d\n", solution_lens[i][0], open_sizes[i]);

            // // fprintf(fout_mem, "solution_len %d open %d closed %d\n", solution_lens[i][1], list_sizes[i][0], list_sizes[i][1]);
            // fclose(fout_mem);
        }

        freeMap(stateMap, row_size);
    }

    return;
}

int main()
{
    vector<string> maps;
    maps.push_back("../maps/random512-10-0.map");
    maps.push_back("../maps/maze512-1-0.map");
    maps.push_back("../maps/brc202d.map");
    maps.push_back("../maps/den012d.map");
    maps.push_back("../maps/orz800d.map");

    test_maps(maps, true, true, true, true, true);

    return 0;
}
