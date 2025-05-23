#include <vector>
#include <iostream>
#include <chrono>
#include <string>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <sys/stat.h>
#include "tiles.h"
#include "ACLS.h"
#include "LACLS.h"
#include "frontier.h"
#include "astar.h"
#include "smgs.h"

using namespace std;

std::vector<TileMap> readTileConfigurations(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open file: " + filename);
    }

    std::vector<TileMap> tile_maps;
    TileMap current_map;
    std::string line;
    int row = 0;

    while (std::getline(file, line))
    {
        // Skip empty lines
        if (line.empty())
        {
            continue;
        }

        // Check for separator line (e.g., "------------------")
        if (line.find('-') != std::string::npos)
        {
            if (!current_map.empty())
            {
                tile_maps.push_back(current_map);
                current_map.clear();
            }
            row = 0; // Reset row for the next configuration
            continue;
        }

        // Parse the current row of the puzzle
        std::istringstream iss(line);
        int value;
        int col = 0;
        while (iss >> value)
        {
            current_map[{row, col}] = value;
            col++;
        }
        row++;
    }

    // Add the last map if not empty
    if (!current_map.empty())
    {
        tile_maps.push_back(current_map);
    }

    file.close();
    return tile_maps;
}

void test_maps(vector<TileMap> tile_maps, bool run_astar, bool run_frontier, bool run_ACLS, bool run_LACLS, bool run_smgs)
{
    int row_size = 4;
    int col_size = 4;

    TileEnvironment TileEnv;
    std::vector<std::pair<int, int>> actions;
    std::vector<double> costs;
    for (std::pair<int, int> action : {make_pair(-1, 0), make_pair(0, -1), make_pair(0, 1), make_pair(1, 0)})
    {
        actions.push_back(action);
        costs.push_back(1);
    }

    // Goal map for random puzzles
    TileMap goal_map({
        {make_pair(0, 0), 1},
        {make_pair(0, 1), 2},
        {make_pair(0, 2), 3},
        {make_pair(0, 3), 4},
        {make_pair(1, 0), 5},
        {make_pair(1, 1), 6},
        {make_pair(1, 2), 7},
        {make_pair(1, 3), 8},
        {make_pair(2, 0), 9},
        {make_pair(2, 1), 10},
        {make_pair(2, 2), 11},
        {make_pair(2, 3), 12},
        {make_pair(3, 0), 13},
        {make_pair(3, 1), 14},
        {make_pair(3, 2), 15},
        {make_pair(3, 3), 0},
    });

    int map_num = 0;
    for (TileMap start_map : tile_maps)
    {

        // Weight for the heuristic
        int weight = 1;

        // Weight for the attractor distance function
        int dist_weight = 1;

        int iters = 1;

        vector<vector<double>> planner_times(iters, vector<double>(4, 0.0));
        vector<vector<int>> solution_lens(iters, vector<int>(4, 0));
        vector<vector<int>> closed_list_size(iters, vector<int>(4, 0));

        printf("----------Current map: %d-----------\n", map_num);
        for (auto p : start_map)
        {
            printf("%d ", p.second);
        }
        printf("\n");

        for (int i = 0; i < iters; i++)
        {

            int start_state_id, goal_state_id;

            vector<int> solution_ids;
            int cost;
            double time_span = 0;
            double astar_time, frontier_time, frontier_attr_time, frontier_attr_lazy_time, frontier_attr_lazy_base_time, SMGS_time = 0;
            chrono::high_resolution_clock::time_point t1, t2;

            if (run_ACLS)
            {
                if (!TileEnv.init(row_size, col_size, actions, costs, "ACLS"))
                {
                    throw runtime_error("Grid Initialization failed");
                }
                TileEnv.setStartState(start_map);
                TileEnv.setGoalState(goal_map);
                TileEnv.setWeight(dist_weight);
                start_state_id = TileEnv.getStartStateID();
                goal_state_id = TileEnv.getGoalStateID();

                ACLS ACLSPlanner(&TileEnv);
                ACLSPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();
                ACLSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);

                std::vector<TileState *> solution_states;
                TileEnv.extractPath("attractors", solution_ids, solution_states);

                t2 = chrono::high_resolution_clock::now();
                frontier_attr_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("ACLS Planner time: %f\n", frontier_attr_time);

                planner_times[i][2] = frontier_attr_time;
                solution_lens[i][2] = (int)solution_ids.size();
                closed_list_size[i][2] = ACLSPlanner.getNumAttractors();

                solution_ids.clear();
                printf("ACLS Num of attractors: %d\n", ACLSPlanner.getNumAttractors());
            }

            if (run_LACLS)
            {
                if (!TileEnv.init(row_size, col_size, actions, costs, "LACLS"))
                {
                    throw runtime_error("Grid Initialization failed");
                }

                TileEnv.setStartState(start_map);
                TileEnv.setGoalState(goal_map);
                TileEnv.setWeight(dist_weight);

                start_state_id = TileEnv.getStartStateID();
                goal_state_id = TileEnv.getGoalStateID();

                LACLS LACLSPlanner(&TileEnv);
                LACLSPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();
                int err = LACLSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                if (err)
                {
                    printf("Planning failed\n");
                }
                std::vector<TileState *> solution_states;
                TileEnv.extractPath("attractors", solution_ids, solution_states);
                t2 = chrono::high_resolution_clock::now();
                frontier_attr_lazy_base_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("LACLS Planner time: %f\n", frontier_attr_lazy_base_time);

                planner_times[i][3] = frontier_attr_lazy_base_time;
                solution_lens[i][3] = (int)solution_ids.size();
                closed_list_size[i][3] = LACLSPlanner.getNumAttractors();

                printf("LACLS Num of attractors: %d\n", LACLSPlanner.getNumAttractors());
                printf("LACLS Pathlen: %d\n", (int)solution_ids.size());
                solution_ids.clear();
            }

            if (run_frontier)
            {
                if (!TileEnv.init(row_size, col_size, actions, costs, "FA*"))
                {
                    throw runtime_error("Grid Initialization failed");
                }
                TileEnv.setStartState(start_map);
                TileEnv.setGoalState(goal_map);
                TileEnv.setWeight(dist_weight);
                start_state_id = TileEnv.getStartStateID();
                goal_state_id = TileEnv.getGoalStateID();
                FrontierAStar FrontierPlanner(&TileEnv);
                FrontierPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();

                FrontierPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                std::vector<TileState *> frontier_solution_states;
                TileEnv.extractPath("frontier", solution_ids, frontier_solution_states);
                t2 = chrono::high_resolution_clock::now();
                frontier_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

                planner_times[i][1] = frontier_time;
                solution_lens[i][1] = (int)solution_ids.size();
                closed_list_size[i][1] = 0;

                printf("Frontier A* Planner time: %f\n", frontier_time);
                printf("Frontier A* Pathlen: %d\n", (int)solution_ids.size());

                solution_ids.clear();
            }

            if (run_astar)
            {
                // A*
                if (!TileEnv.init(row_size, col_size, actions, costs, "A*"))
                {
                    throw runtime_error("Grid Initialization failed");
                }
                TileEnv.setStartState(start_map);
                TileEnv.setGoalState(goal_map);
                TileEnv.setWeight(dist_weight);

                start_state_id = TileEnv.getStartStateID();
                goal_state_id = TileEnv.getGoalStateID();
                AStar AStarPlanner(&TileEnv);
                AStarPlanner.setWeight(weight);
                t1 = chrono::high_resolution_clock::now();
                AStarPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                std::vector<TileState *> astar_solution_states;
                TileEnv.extractPath("astar", solution_ids, astar_solution_states);

                t2 = chrono::high_resolution_clock::now();
                astar_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("A* Planner time: %f\n", astar_time);
                printf("A* Planner expansions: %d\n", AStarPlanner.getExpansions());

                planner_times[i][0] = astar_time;
                solution_lens[i][0] = (int)solution_ids.size();
                closed_list_size[i][0] = AStarPlanner.getExpansions();

                printf("A* Pathlen: %d\n", (int)solution_ids.size());

                solution_ids.clear();
            }

            if (run_smgs)
            {
                if (!TileEnv.init(row_size, col_size, actions, costs, "smgs"))
                {
                    throw runtime_error("Grid Initialization failed");
                }
                TileEnv.setStartState(start_map);
                TileEnv.setGoalState(goal_map);
                TileEnv.setWeight(dist_weight);

                start_state_id = TileEnv.getStartStateID();
                goal_state_id = TileEnv.getGoalStateID();
                SMGS SMGSPlanner(&TileEnv);
                SMGSPlanner.setWeight(weight);
                SMGSPlanner.setClosedThreshold(closed_list_size[i][0] * 0.1);
                SMGSPlanner.setClosedMaxThreshold(closed_list_size[i][0] * 0.5); // Set to 0.5*A* closed size
                t1 = chrono::high_resolution_clock::now();
                SMGSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                std::vector<TileState *> SMGS_solution_states;
                TileEnv.extractPath("SMGS", solution_ids, SMGS_solution_states);
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
            // string file = "planner_times_tiles.txt";
            // FILE *fout = fopen(file.c_str(), "a");
            // fprintf(fout, "solution_len %d closed: %d %d %d %d %d time %f %f %f %f %f\n", solution_lens[i][0], closed_list_size[i][0], closed_list_size[i][1], closed_list_size[i][2], closed_list_size[i][3], closed_list_size[i][4], planner_times[i][0], planner_times[i][1], planner_times[i][2], planner_times[i][3], planner_times[i][4]);
            // fclose(fout);
        }

        // Next map
        map_num++;
    }

    return;
}

int main()
{

    vector<TileMap> tile_maps;
    string filename = "../15_tile_map.txt";
    tile_maps = readTileConfigurations(filename);

    // Print the tile maps
    // for (size_t i = 0; i < tile_maps.size(); ++i)
    // {
    //     std::cout << "Configuration " << i + 1 << ":\n";
    //     for (const auto &entry : tile_maps[i])
    //     {
    //         std::cout << "(" << entry.first.first << ", " << entry.first.second << ") -> " << entry.second << "\n";
    //     }
    //     std::cout << "------------------\n";
    // }
    // getc(stdin);
    test_maps(tile_maps, true, true, true, true, false);

    return 0;
}
