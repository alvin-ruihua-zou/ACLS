#include <vector>
#include <iostream>
#include <chrono>
#include <string>
#include <filesystem>
#include <sys/stat.h>
#include "tower.h"
#include "ACLS.h"
#include "LACLS.h"
#include "frontier.h"
#include "astar.h"
#include "smgs.h"

using namespace std;

void test_maps(vector<pair<int, int>> tower_maps, bool run_astar, bool run_frontier, bool run_ACLS, bool run_LACLS, bool run_smgs)
{

    int map_num = 0;
    for (pair<int, int> tower_map : tower_maps)
    {
        int disc_num = tower_map.first;
        int peg_num = tower_map.second;
        printf("Pegs: %d, discs: %d\n", peg_num, disc_num);
        TowerEnvironment TowerEnv;
        std::vector<std::pair<int, int>> actions;
        std::vector<double> costs;

        // Generate actions and costs
        for (int peg = 0; peg < peg_num; peg++)
        {
            for (int peg2 = peg + 1; peg2 < peg_num; peg2++)
            {
                actions.push_back(std::make_pair(peg, peg2));
                costs.push_back(1);
            }
        }
        // Generate the second half by mirroring the first half and reversing pairs
        for (int i = actions.size() - 1; i >= 0; i--)
        {
            actions.push_back(std::make_pair(actions[i].second, actions[i].first));
            costs.push_back(1);
        }

        // Initialize the start and goal states
        TowerPegs start_state(peg_num);
        for (int disc = disc_num; disc > 0; disc--)
        {
            start_state[0].push_back(disc);
        }

        TowerPegs goal_state(peg_num);
        for (int disc = disc_num; disc > 0; disc--)
        {
            goal_state[peg_num - 1].push_back(disc);
        }

        int iters = 1;

        // Weight for the heuristic
        int weight = 1;

        // Weight for the attractor distance function
        int dist_weight = 1;

        vector<vector<double>> planner_times(iters, vector<double>(4, 0.0));
        vector<vector<int>> solution_lens(iters, vector<int>(4, 0));
        vector<vector<int>> closed_list_size(iters, vector<int>(4, 0));

        for (int i = 0; i < iters; i++)
        {

            int start_state_id, goal_state_id;

            vector<int> solution_ids;
            int cost;
            double time_span = 0;
            double astar_time, frontier_time, ACLS_time, LACLS_time, SMGS_time = 0;
            chrono::high_resolution_clock::time_point t1, t2;

            if (run_ACLS)
            {
                if (!TowerEnv.init(disc_num, peg_num, actions, costs, "ACLS"))
                {
                    throw runtime_error("Map Initialization failed");
                }
                TowerEnv.setStartState(start_state);
                TowerEnv.setGoalState(goal_state);
                TowerEnv.setWeight(dist_weight);
                start_state_id = TowerEnv.getStartStateID();
                goal_state_id = TowerEnv.getGoalStateID();

                ACLS ACLSPlanner(&TowerEnv);
                t1 = chrono::high_resolution_clock::now();
                ACLSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);

                std::vector<TowerState *> solution_states;
                TowerEnv.extractPath("attractors", solution_ids, solution_states);

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
                if (!TowerEnv.init(disc_num, peg_num, actions, costs, "LACLS"))
                {
                    throw runtime_error("Map Initialization failed");
                }

                TowerEnv.setStartState(start_state);
                TowerEnv.setGoalState(goal_state);
                TowerEnv.setWeight(dist_weight);
                start_state_id = TowerEnv.getStartStateID();
                goal_state_id = TowerEnv.getGoalStateID();

                LACLS LACLSPlanner(&TowerEnv);
                t1 = chrono::high_resolution_clock::now();
                int err = LACLSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                if (err)
                {
                    printf("Planning failed\n");
                }
                std::vector<TowerState *> solution_states;
                TowerEnv.extractPath("attractors", solution_ids, solution_states);

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
                if (!TowerEnv.init(disc_num, peg_num, actions, costs, "FA*"))
                {
                    throw runtime_error("Map Initialization failed");
                }
                TowerEnv.setStartState(start_state);
                TowerEnv.setGoalState(goal_state);
                TowerEnv.setWeight(dist_weight);
                start_state_id = TowerEnv.getStartStateID();
                goal_state_id = TowerEnv.getGoalStateID();
                FrontierAStar FrontierPlanner(&TowerEnv);
                t1 = chrono::high_resolution_clock::now();

                FrontierPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);

                std::vector<TowerState *> frontier_solution_states;
                TowerEnv.extractPath("frontier", solution_ids, frontier_solution_states);
                t2 = chrono::high_resolution_clock::now();
                frontier_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("Frontier A* Planner time: %f\n", frontier_time);
                printf("Fontier A* Pathlen: %d\n", (int)solution_ids.size());

                planner_times[i][1] = frontier_time;
                solution_lens[i][1] = (int)solution_ids.size();
                closed_list_size[i][1] = 0;

                solution_ids.clear();
            }

            if (run_astar)
            {
                if (!TowerEnv.init(disc_num, peg_num, actions, costs, "A*"))
                {
                    throw runtime_error("Map Initialization failed");
                }
                TowerEnv.setStartState(start_state);
                TowerEnv.setGoalState(goal_state);
                TowerEnv.setWeight(dist_weight);
                start_state_id = TowerEnv.getStartStateID();
                goal_state_id = TowerEnv.getGoalStateID();
                AStar AStarPlanner(&TowerEnv);
                t1 = chrono::high_resolution_clock::now();

                AStarPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                std::vector<TowerState *> astar_solution_states;
                TowerEnv.extractPath("astar", solution_ids, astar_solution_states);
                t2 = chrono::high_resolution_clock::now();
                astar_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();
                printf("A* Planner time: %f\n", astar_time);
                printf("A* Planner expansions: %d\n", AStarPlanner.getExpansions());
                printf("A* Pathlen: %d\n", (int)solution_ids.size());

                planner_times[i][0] = astar_time;
                solution_lens[i][0] = (int)solution_ids.size();
                closed_list_size[i][0] = AStarPlanner.getExpansions();

                solution_ids.clear();
            }
            if (run_smgs)
            {
                if (!TowerEnv.init(disc_num, peg_num, actions, costs, "smgs"))
                {
                    throw runtime_error("Map Initialization failed");
                }
                TowerEnv.setStartState(start_state);
                TowerEnv.setGoalState(goal_state);
                TowerEnv.setWeight(dist_weight);

                start_state_id = TowerEnv.getStartStateID();
                goal_state_id = TowerEnv.getGoalStateID();
                SMGS SMGSPlanner(&TowerEnv);
                SMGSPlanner.setWeight(weight);
                SMGSPlanner.setClosedThreshold(closed_list_size[i][0] * 0.1);
                SMGSPlanner.setClosedMaxThreshold(closed_list_size[i][0] * 0.5); // Set to 0.5*A* closed size
                t1 = chrono::high_resolution_clock::now();
                SMGSPlanner.plan(start_state_id, goal_state_id, &solution_ids, &cost);
                std::vector<TowerState *> SMGS_solution_states;
                TowerEnv.extractPath("SMGS", solution_ids, SMGS_solution_states);

                t2 = chrono::high_resolution_clock::now();
                SMGS_time = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

                planner_times[i][4] = SMGS_time;
                solution_lens[i][4] = (int)solution_ids.size();
                closed_list_size[i][4] = SMGSPlanner.getClosedMaxSize();

                printf("SMGS Planner time: %f\n", SMGS_time);
                printf("SMGS Closed list size: %d\n", SMGSPlanner.getClosedMaxSize());
                printf("SMGS Pathlen: %d\n", (int)solution_ids.size());
                solution_ids.clear();
            }
            printf("\n\n");

            // For recording the results
            // string file = "planner_times_tower_w=" + to_string(weight) + ".txt";
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

    vector<pair<int, int>> tower_maps;
    for (int i = 3; i < 12; i++)
    {
        tower_maps.push_back(make_pair(i, 3));
    }

    test_maps(tower_maps, true, true, true, true, true);

    return 0;
}

// g++ -o tower.out tower.h tower.cpp compares.h compares.cpp smgs.h smgs.cpp ACLS.h ACLS.cpp LACLS.h LACLS.cpp frontier.h frontier.cpp astar.h astar.cpp test_tower_puzzle.cpp