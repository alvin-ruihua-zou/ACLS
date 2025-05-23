#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include "frontier.h"
#include "compares.h"
using namespace std;

FrontierAStar::FrontierAStar(DiscreteSpaceInformation *space)
    : m_space(space),
      m_open(),
      m_states(),
      m_midpoint_states(),
      m_start_state_id(-1),
      m_goal_state_id(-1),
      m_init_goal_state(-1),
      m_weight(1),
      expansions(0),
      m_succs(),
      m_costs()
{
}

FrontierAStar::~FrontierAStar()
{
    for (SearchState *s : m_states)
    {
        if (s != NULL)
        {
            delete s;
        }
    }

    for (pair<int, SearchState *> s : m_midpoint_states)
    {
        delete s.second;
    }
    m_open.clear();
    m_states.clear();
    m_midpoint_states.clear();
}

void FrontierAStar::printState(int state_id)
{
    SearchState *s = m_states[state_id];
    if (s == nullptr || s == NULL)
    {
        printf("midpoint: ");
        s = m_midpoint_states[state_id];
    }
    if (s == nullptr || s == NULL)
    {
        printf("temp midpoint: ");
        s = m_temp_midpoint_states[state_id];
    }
    if (s == nullptr || s == NULL)
    {
        printf("invalid state: %d\n", state_id);
        return;
    }
    printf("StateID: %d, g: %f, h: %f, ", s->state_id, s->g, s->h);
    if (s->midpoint_state == NULL)
    {
        printf("midpoint: NULL, operators: ");
    }
    else
    {
        printf("midpoint: %d, operators: ", s->midpoint_state->state_id);
    }
    for (int i = 0; i < NUM_OF_ACTIONS; i++)
    {
        printf(" %d", s->action_operators.test(i));
    }

    printf("\n");
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
FrontierAStar::SearchState *FrontierAStar::getSearchState(int state_id)
{
    if (m_states.size() <= state_id)
    {
        m_states.resize(state_id + 1, nullptr);
    }

    auto &state = m_states[state_id];
    if (state == NULL)
    {
        state = createState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
FrontierAStar::SearchState *FrontierAStar::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState *ss = new SearchState;
    ss->state_id = state_id;
    initSearchState(ss);

    return ss;
}

// Initialize a search state.
void FrontierAStar::initSearchState(SearchState *state)
{

    state->g = INFINITECOST;
    state->h = m_space->GetGoalHeuristic(state->state_id);
    for (size_t i = 0; i < state->action_operators.size(); i++)
    {
        (state->action_operators).set(i);
    }
    state->midpoint_state = nullptr;
}

double FrontierAStar::computeFval(SearchState *s) const
{
    return s->g + s->h * m_weight;
}

bool FrontierAStar::almostEqual(double c1, double c2)
{
    if (abs(c1 - c2) < 1e-6)
    {
        return true;
    }
    return false;
}

// Expand a state, updating its successors and placing them into OPEN or ATTRACTORS as appropriate
void FrontierAStar::expand(SearchState *s)
{

    m_succs.clear();
    m_costs.clear();
    m_action_ids.clear();
    m_space->GetSuccs(s->state_id, s->action_operators, &m_succs, &m_costs, &m_action_ids);

    for (size_t idx = 0; idx < m_succs.size(); ++idx)
    {

        int succ_state_id = m_succs[idx];
        int cost = m_costs[idx];
        int action_id = m_action_ids[idx];

        // Must check if the operator for the succ_state is valid.
        if (succ_state_id == -1)
        {
            continue;
        }

        SearchState *succ_state = getSearchState(succ_state_id);

        // Update operators
        succ_state->action_operators.reset(succ_state->action_operators.size() - action_id - 1);

        int new_cost = s->g + cost;

        // If succ_state not in open its cost is INFINITY
        // So condition checks (if succ_state is in OPEN and better cost) or (not in open)
        if (new_cost < succ_state->g)
        {
            succ_state->g = new_cost;
            succ_state->f = computeFval(succ_state);

            if (m_open.contains(succ_state))
            {
                m_open.decrease(succ_state);
            }
            else
            {
                m_open.push(succ_state);
            }
        }
        // Don't do anything if cost is worse
    }
}

enum PlanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    EXHAUSTED_OPEN_LIST
};

int FrontierAStar::getPathCost(SearchState *goal_state, int *cost)
{
    while (!m_open.empty())
    {
        SearchState *min_state = m_open.min();
        // path to goal found
        if (min_state->state_id == goal_state->state_id || min_state->g == goal_state->g)
        {

            *cost = min_state->g;
            return SUCCESS;
        }

        m_open.pop();

        expand(min_state);

        // No need for closed list
        m_states[min_state->state_id] = nullptr;
        m_space->DeleteState(min_state->state_id);
        delete min_state;
    }
    printf("Failed to find path\n");
    return EXHAUSTED_OPEN_LIST;
}

FrontierAStar::SearchState *FrontierAStar::getMidpointState(SearchState *start_state, SearchState *goal_state, int midline_cost, int *goal_cost)
{

    while (!m_open.empty())
    {

        SearchState *min_state = m_open.min();

        // path to goal found
        if (min_state->state_id == goal_state->state_id || min_state->g == goal_state->g) //|| min_state->g == goal_state->g
        {

            // if (goal_state->midpoint_state == NULL)
            // {
            //     for (auto p : m_temp_midpoint_states)
            //     {
            //         SearchState *s = p.second;
            //         delete s;
            //         // if (s != NULL && s->state_id != midpoint_state->state_id)
            //         // {
            //         //     m_temp_midpoint_states[s->state_id] = nullptr;
            //         //     delete s;
            //         // }
            //     }

            //     m_temp_midpoint_states.clear();
            //     m_space->ClearTempMidpoints();

            //     // m_open.pop();
            //     // m_space->DeleteState(min_state->state_id);
            //     // delete min_state;
            //     m_open.clear();
            //     return nullptr;
            // }

            *goal_cost = min_state->g;
            SearchState *midpoint_state = new SearchState;

            midpoint_state->state_id = goal_state->midpoint_state->state_id;
            midpoint_state->f = goal_state->midpoint_state->f;
            midpoint_state->g = goal_state->midpoint_state->g;
            midpoint_state->h = goal_state->midpoint_state->h;
            midpoint_state->midpoint_state = midpoint_state;
            for (size_t i = 0; i < midpoint_state->action_operators.size(); i++)
            {
                (midpoint_state->action_operators).set(i);
            }

            m_space->AddMidpointState(midpoint_state->state_id, m_midpoint_states.size());
            midpoint_state->state_id = m_midpoint_states.size();
            m_midpoint_states[midpoint_state->state_id] = midpoint_state;

            while (!m_open.empty())
            {
                min_state = m_open.min();
                m_open.pop();
                if (min_state->midpoint_state != nullptr && m_temp_midpoint_states.count(min_state->midpoint_state->state_id) == 0 && min_state->midpoint_state->state_id != midpoint_state->state_id)
                {
                    delete min_state->midpoint_state;
                    min_state->midpoint_state = nullptr;
                }
                if (min_state->state_id != midpoint_state->state_id)
                {
                    m_states[min_state->state_id] = nullptr;
                    m_space->DeleteState(min_state->state_id);
                    delete min_state;
                }
            }

            for (auto p : m_temp_midpoint_states)
            {
                SearchState *s = p.second;
                delete s;
            }

            m_temp_midpoint_states.clear();
            m_space->ClearTempMidpoints();

            m_open.clear();

            return midpoint_state;
        }

        m_open.pop();
        assert(min_state->g != INFINITECOST);

        // Expand
        m_succs.clear();
        m_costs.clear();
        m_action_ids.clear();
        SearchState *s = min_state;

        m_space->GetSuccs(s->state_id, s->action_operators, &m_succs, &m_costs, &m_action_ids);
        for (size_t idx = 0; idx < m_succs.size(); ++idx)
        {

            int succ_state_id = m_succs[idx];
            int cost = m_costs[idx];
            int action_id = m_action_ids[idx];

            // Must check if the operator for the succ_state is valid.
            if (succ_state_id == -1)
            {
                continue;
            }

            SearchState *succ_state = getSearchState(succ_state_id);

            // Update operators
            succ_state->action_operators.reset(succ_state->action_operators.size() - action_id - 1);

            int new_cost = s->g + cost;
            // If succ_state not in open its cost is INFINITY
            // So condition checks (if succ_state is in OPEN and better cost) or (not in open)
            if (new_cost < succ_state->g)
            {
                succ_state->g = new_cost;
                succ_state->f = computeFval(succ_state);
                if ((succ_state->g >= midline_cost)) //(succ_state->g >= midline_cost) !isLessThan(succ_state->g, midline_cost)
                {
                    if (s->midpoint_state == NULL)
                    {
                        SearchState *ss = new SearchState;
                        ss->state_id = succ_state->state_id;
                        ss->f = succ_state->f;
                        ss->g = succ_state->g;
                        ss->midpoint_state = ss;
                        for (size_t i = 0; i < ss->action_operators.size(); i++)
                        {
                            (ss->action_operators).set(i);
                        }
                        if (m_temp_midpoint_states[ss->state_id] != NULL)
                        {
                            delete m_temp_midpoint_states[ss->state_id];
                            m_temp_midpoint_states[ss->state_id] = nullptr;
                        }
                        m_temp_midpoint_states[ss->state_id] = ss;
                        m_space->AddTempMidpointState(ss->state_id);
                        succ_state->midpoint_state = ss;
                    }
                    else
                    {
                        succ_state->midpoint_state = s->midpoint_state;
                    }
                }
                if (m_open.contains(succ_state))
                {
                    m_open.decrease(succ_state);
                }
                else
                {
                    m_open.push(succ_state);
                }
            }
        }
        // No need for closed list
        m_states[min_state->state_id] = nullptr;
        m_space->DeleteState(min_state->state_id);
        delete min_state;
    }

    printf("Failed to find path\n");
    return nullptr;
}

void FrontierAStar::extractPath(
    SearchState *start_state,
    SearchState *goal_state,
    int midline_cost,
    std::vector<int> &solution,
    int &cost, int depth)
{

    bool neighbor = m_space->isNeighbor("frontier", start_state->state_id, goal_state->state_id);

    if (neighbor)
    {

        solution.push_back((goal_state->state_id));
        cost = goal_state->g;

        return;
    }

    m_start_state_id = start_state->state_id;
    m_goal_state_id = goal_state->state_id;

    m_space->reinit_frontier_search(start_state->state_id, goal_state->state_id);

    SearchState *search_goal_state = getSearchState(m_goal_state_id);
    SearchState *search_start_state = getSearchState(m_start_state_id);

    search_start_state->g = 0;
    search_start_state->f = computeFval(search_start_state);
    assert(m_open.empty());
    m_open.push(search_start_state);

    int goal_cost;
    SearchState *midpoint_state = getMidpointState(search_start_state, search_goal_state, midline_cost, &goal_cost);

    m_space->DeleteState(m_goal_state_id);

    delete m_states[m_goal_state_id];
    m_states[m_goal_state_id] = nullptr;
    for (SearchState *s : m_states)
    {
        if (s != NULL)
        {

            m_space->DeleteState(s->state_id);

            m_states[s->state_id] = nullptr;
            delete s;
        }
    }
    m_states.clear();
    m_open.clear();

    // if (midpoint_state == nullptr)
    // {
    //     extractPath(start_state, goal_state, midline_cost / 2, solution, cost, depth + 1);
    //     return;
    // }
    // extractPath(start_state, midpoint_state, max(1, (int)floor((midline_cost) / 2.0)), solution, cost, depth + 1);
    // extractPath(midpoint_state, goal_state, max(1, (int)floor((midline_cost) / 2.0)), solution, cost, depth + 1);

    extractPath(start_state, midpoint_state, max(1, (int)floor((midpoint_state->g) / 2.0)), solution, cost, depth + 1);
    extractPath(midpoint_state, goal_state, max(1, (int)floor((goal_cost - midpoint_state->g) / 2.0)), solution, cost, depth + 1);
}

int FrontierAStar::plan(int startID, int goalID, vector<int> *solution, int *cost)
{

    m_start_state_id = startID;
    m_goal_state_id = goalID;
    m_init_goal_state = goalID;

    SearchState *start_state = getSearchState(m_start_state_id);
    SearchState *goal_state = getSearchState(m_goal_state_id);

    // For recovering solution
    m_space->AddMidpointState(start_state->state_id, start_state->state_id);
    m_space->AddMidpointState(goal_state->state_id, goal_state->state_id);

    start_state->g = 0;
    start_state->f = computeFval(start_state);
    m_open.push(start_state);

    int path_cost;
    int err = getPathCost(goal_state, &path_cost);

    if (err != SUCCESS)
    {
        return err;
    }

    m_open.clear();
    // Reinit start and goal states
    for (SearchState *s : m_states)
    {
        if (s != NULL)
        {
            m_space->DeleteState(s->state_id);
            delete s;
        }
    }

    m_states.clear();

    // Can't use createState since states don't exist in the environment (getHeuristic will fail)
    start_state = new SearchState;
    start_state->state_id = m_start_state_id;
    start_state->g = 0;

    goal_state = new SearchState;
    goal_state->state_id = m_goal_state_id;
    goal_state->g = path_cost;

    m_midpoint_states[start_state->state_id] = start_state;
    m_midpoint_states[goal_state->state_id] = goal_state;
    solution->push_back((start_state->state_id));

    extractPath(start_state, goal_state, max(1, (int)floor((path_cost) / 2.0)), *solution, *cost, 0);

    return SUCCESS;
}
