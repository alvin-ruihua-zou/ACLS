#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
// #include <gperftools/heap-profiler.h>
#include "astar.h"
using namespace std;

/**
 * @brief Constructor for the AStar class.
 * @param space Pointer to the discrete space environment.
 */
AStar::AStar(DiscreteSpaceInformation *space)
    : m_space(space),
      m_open(),
      m_states(),
      m_start_state_id(-1),
      m_goal_state_id(-1),
      m_weight(1),
      m_succs(),
      m_costs(),
      expansions(0)
{
}

/**
 * @brief Destructor for the AStar class.
 */
AStar::~AStar()
{
    for (SearchState *s : m_states)
    {
        if (s != NULL)
        {
            delete s;
        }
    }
    m_states.clear();
}

/**
 * @brief Prints information about a specific state.
 * @param state_id The ID of the state to print.
 */
void AStar::printState(int state_id)
{
    SearchState *s = m_states[state_id];
    printf("StateID: %d, g: %f, h: %f, bp_state: %d", s->state_id, s->g, s->h, (s != nullptr) ? s->bp_state->state_id : -1);
    printf("\n");
}

/**
 * @brief Retrieves the search state corresponding to a graph state, creating a new state if
 * one has not been created yet.
 * @param state_id The ID of the graph state.
 * @return Pointer to the corresponding search state.
 */
AStar::SearchState *AStar::getSearchState(int state_id)
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

/**
 * @brief Creates a new search state for a graph state.
 * @param state_id The ID of the graph state.
 * @return Pointer to the newly created search state.
 */
AStar::SearchState *AStar::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState *ss = new SearchState;
    ss->state_id = state_id;
    initSearchState(ss);

    return ss;
}

/**
 * @brief Initializes a search state.
 * @param state Pointer to the search state to initialize.
 */
void AStar::initSearchState(SearchState *state)
{

    state->g = INFINITECOST;
    state->h = m_space->GetGoalHeuristic(state->state_id);

    for (size_t i = 0; i < state->action_operators.size(); i++)
    {
        (state->action_operators).set(i);
    }
}

/**
 * @brief Computes the f-value for a given state.
 * @param s Pointer to the search state.
 * @return The computed f-value.
 */
double AStar::computeFval(SearchState *s) const
{
    return s->g + s->h * m_weight;
}

/**
 * @brief Checks if two floating-point numbers are approximately equal.
 * @param c1 The first number.
 * @param c2 The second number.
 * @return True if the numbers are approximately equal, false otherwise.
 */
bool AStar::almostEqual(double c1, double c2)
{
    if (abs(c1 - c2) < 1e-6)
    {
        return true;
    }
    return false;
}

/**
 * @brief Expands a search state, updating its successors.
 * @param s Pointer to the search state to expand.
 */
void AStar::expand(SearchState *s)
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

        succ_state->action_operators.reset(succ_state->action_operators.size() - action_id - 1);

        int new_cost = s->g + cost;

        // If succ_state not in open its cost is INFINITY
        // So condition checks (if succ_state is in OPEN and better cost) or (not in open)

        if (new_cost < succ_state->g)
        {

            succ_state->g = new_cost;
            succ_state->f = computeFval(succ_state);

            succ_state->bp_state = s;

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

/**
 * @brief Retrieves the path to the goal state.
 * @param goal_state Pointer to the goal state.
 * @return Status code indicating success or failure.
 */
int AStar::getPath(SearchState *goal_state)
{

    while (!m_open.empty())
    {

        SearchState *min_state = m_open.min();

        // If OPEN contains multiple states with smallest g value and
        // s_goal is one of them, pop s_goal

        if (min_state->state_id == goal_state->state_id || min_state->g == goal_state->g)
        {

            return SUCCESS;
        }

        m_open.pop();

        expand(min_state);

        expansions += 1;
    }
    printf("Failed to find path\n");
    return EXHAUSTED_OPEN_LIST;
}

/**
 * @brief Extracts the path from the start state to the goal state.
 * @param start_state Pointer to the start state.
 * @param goal_state Pointer to the goal state.
 * @param solution Vector to store the solution path.
 * @param cost Reference to store the total cost of the path.
 */
void AStar::extractPath(
    SearchState *start_state,
    SearchState *goal_state,
    std::vector<int> &solution,
    int &cost)
{
    SearchState *curr_state = goal_state;
    solution.push_back(goal_state->state_id);
    while (curr_state->state_id != start_state->state_id)
    {
        curr_state = curr_state->bp_state;
        solution.push_back(curr_state->state_id);
    }
    std::reverse(solution.begin(), solution.end());
}

/**
 * @brief Plans a path from the start state to the goal state.
 * @param startID The ID of the start state.
 * @param goalID The ID of the goal state.
 * @param solution Pointer to store the solution path.
 * @param cost Pointer to store the total cost of the path.
 * @return Status code indicating success or failure.
 */
int AStar::plan(int startID, int goalID, vector<int> *solution, int *cost)
{

    m_open.clear();
    m_states.clear();

    m_start_state_id = startID;
    m_goal_state_id = goalID;

    SearchState *start_state = getSearchState(m_start_state_id);
    SearchState *goal_state = getSearchState(m_goal_state_id);

    start_state->g = 0;
    start_state->f = computeFval(start_state);
    m_open.push(start_state);
    int err = getPath(goal_state);

    if (err != SUCCESS)
    {
        return err;
    }

    extractPath(start_state, goal_state, *solution, *cost);

    return SUCCESS;
}

/**
 * @brief Retrieves the number of state expansions performed.
 * @return The number of state expansions.
 */
int AStar::getExpansions()
{
    return expansions;
}
