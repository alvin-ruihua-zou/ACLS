#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include "ACLS.h"

ACLS::ACLS(DiscreteSpaceInformation *space)
    : m_space(space),
      m_open(),
      m_attractors(),
      m_states(),
      m_weight(1),
      expansions(0),
      m_start_state_id(-1),
      m_goal_state_id(-1),
      m_succs(),
      m_costs()
{
}

ACLS::~ACLS()
{
    for (SearchState *s : m_states)
    {
        if (s != NULL && s != nullptr)
        {
            delete s;
        }
    }

    for (Attractor *a : m_attractors)
    {
        if (a != NULL)
        {
            delete a;
        }
    }
    m_attractors.clear();
    m_states.clear();
}

void ACLS::printState(int state_id)
{
    if (state_id >= m_states.size() || state_id < 0)
    {
        printf("Invalid state id\n");
        return;
    }
    SearchState *s = m_states[state_id];
    printf("StateID: %d, g: %f, h: %f, attr_id: %d, operators: ", s->state_id, s->g, s->h, s->attr_id);
    for (int i = 0; i < NUM_OF_ACTIONS; i++)
    {
        printf(" %d", s->action_operators.test(i));
    }

    printf("\n");
}

void ACLS::printStates()
{
    printf("---------Printing States-----------\n");
    SearchState *state;
    auto itr = m_states.begin();
    while (itr != m_states.end())
    {
        state = *itr;
        if (state != NULL)
        {
            printState(state->state_id);
            m_space->printState(state->state_id);
        }

        itr++;
    }
    printf("------------Printing complete-------------\n");
}

void ACLS::printAttractor(int attr_id)
{
    Attractor *s = m_attractors[attr_id];
    printf("AttrID: %d, counter: %d, prev_attr_id: %d\n", s->attr_id, s->counter, s->pred_attr_id);
}

void ACLS::printAttractors()
{

    printf("---------Printing attractors-----------\n");
    for (int i = 0; i < m_attractors.size(); i++)
    {
        if (m_attractors[i] != NULL)
        {
            printf("Key: %d ", i);
            printAttractor(m_attractors[i]->attr_id);
        }
    }
    printf("------------Printing complete-------------\n");
}

void ACLS::removeAttractor(int attr_id)
{
    int next_id = m_attractors[attr_id]->pred_attr_id;
    int curr_id = next_id;
    delete m_attractors[attr_id];
    m_attractors[attr_id] = nullptr;
    // m_attractors.erase(attr_id);
    m_space->removeAttractor(attr_id);
    if (next_id == -1 || next_id == attr_id)
    {
        return;
    }
    m_attractors[next_id]->counter -= 1;
    while (m_attractors[next_id]->counter == 0)
    {
        next_id = m_attractors[curr_id]->pred_attr_id;

        delete m_attractors[curr_id];
        m_attractors[curr_id] = nullptr;
        // m_attractors.erase(curr_id);
        m_space->removeAttractor(curr_id);
        if (next_id == -1 || next_id == curr_id)
        {
            return;
        }
        m_attractors[next_id]->counter -= 1;
        curr_id = next_id;
    }
}

int ACLS::getNumAttractors()
{
    int count = 0;
    for (auto *a : m_attractors)
    {
        if (a != NULL)
        {
            count++;
        }
    }
    return count;
}

int ACLS::getExpansions()
{
    return expansions;
}

/// Set the goal state.
int ACLS::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int ACLS::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
ACLS::SearchState *ACLS::getSearchState(int state_id)
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
ACLS::SearchState *ACLS::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState *ss = new SearchState;
    ss->state_id = state_id;
    initSearchState(ss);

    return ss;
}

// Initialize a search state.
void ACLS::initSearchState(SearchState *state)
{

    state->g = INFINITECOST;
    state->h = m_space->GetGoalHeuristic(state->state_id);
    state->attr_id = -1;
    for (size_t i = 0; i < state->action_operators.size(); i++)
    {
        (state->action_operators).set(i);
    }
}

ACLS::Attractor *ACLS::createAttractor(SearchState *s)
{

    Attractor *a = new Attractor;
    a->attr_id = s->state_id;
    a->counter = 0;
    a->pred_attr_id = s->attr_id;

    return a;
}

double ACLS::computeFval(SearchState *s) const
{
    return s->g + s->h * m_weight;
}

bool ACLS::almostEqual(double c1, double c2)
{
    if (abs(c1 - c2) < 1e-6)
    {
        return true;
    }
    return false;
}

// Expand a state, updating its successors and placing them into OPEN or ATTRACTORS as appropriate
void ACLS::expand(SearchState *s)
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
        if (new_cost < succ_state->g)
        {

            succ_state->g = new_cost;
            succ_state->f = computeFval(succ_state);

            if (succ_state->attr_id != s->attr_id)
            {

                // Decrease counter for old attractor
                if (succ_state->attr_id != -1)
                {
                    m_attractors[succ_state->attr_id]->counter -= 1;
                    if (m_attractors[succ_state->attr_id]->counter == 0)
                    {
                        removeAttractor(succ_state->attr_id);
                    }
                }

                int best_pred_state_id = m_space->GetBestPred(succ_state->state_id, s->attr_id);

                if (best_pred_state_id >= 0 && getSearchState(best_pred_state_id) == s)
                {
                    succ_state->attr_id = s->attr_id;
                    m_attractors[s->attr_id]->counter += 1;
                }
                else
                {

                    if (s->state_id >= m_attractors.size())
                    {
                        m_attractors.resize(s->state_id + 1, nullptr);
                    }
                    auto itr = m_attractors[s->state_id];
                    if (itr == NULL)
                    {
                        Attractor *attr = createAttractor(s);
                        m_space->createAttractor(-1, s->state_id);
                        attr->counter = 1;
                        succ_state->attr_id = attr->attr_id;

                        m_attractors[attr->attr_id] = attr;
                    }
                    else
                    {
                        itr->counter += 1;
                        succ_state->attr_id = itr->attr_id;
                    }
                    m_attractors[s->attr_id]->counter += 1;
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
        // succ_state in OPEN but same cost
        else if (almostEqual(new_cost, succ_state->g) && m_space->GetAttractorDist(succ_state->state_id, s->attr_id) > m_space->GetAttractorDist(succ_state->state_id, succ_state->attr_id))
        {

            int best_pred_state_id = m_space->GetBestPred(succ_state->state_id, s->attr_id);
            // s has a better attractor (farther away)
            if (best_pred_state_id >= 0 && getSearchState(best_pred_state_id) == s)
            {
                m_attractors[succ_state->attr_id]->counter -= 1;
                m_attractors[s->attr_id]->counter += 1;
                // Remove attractor if it is not referenced
                if (m_attractors[succ_state->attr_id]->counter == 0)
                {
                    removeAttractor(succ_state->attr_id);
                }
                succ_state->attr_id = s->attr_id;
            }
        }

        // Don't do anything if cost is worse
    }

    m_attractors[s->attr_id]->counter -= 1;
    if (m_attractors[s->attr_id]->counter == 0)
    {
        removeAttractor(s->attr_id);
    }

    m_space->printAttractors();
}

enum PlanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    EXHAUSTED_OPEN_LIST
};

int ACLS::getPath(SearchState *goal_state)
{

    while (!m_open.empty())
    {
        SearchState *min_state = m_open.min();

        // // path to goal found
        // if (min_state->state_id == goal_state->state_id)
        // {
        //     m_open.pop();
        //     while (!m_open.empty())
        //     {
        //         min_state = m_open.min();
        //         m_open.pop();
        //         m_attractors[min_state->attr_id]->counter -= 1;
        //         if (m_attractors[min_state->attr_id]->counter == 0)
        //         {
        //             removeAttractor(min_state->attr_id);
        //         }
        //     }

        //     return SUCCESS;
        // }

        // // Optimization: if OPEN contains multiple states with smallest g value and
        // // s_goal is one of them, pop s_goal
        // if (min_state->g == goal_state->g)
        // {
        //     m_open.pop();
        //     m_attractors[min_state->attr_id]->counter -= 1;
        //     if (m_attractors[min_state->attr_id]->counter == 0)
        //     {
        //         removeAttractor(min_state->attr_id);
        //     }

        //     if (min_state != goal_state)
        //     {
        //         m_attractors[min_state->attr_id]->counter -= 1;
        //         if (m_attractors[min_state->attr_id]->counter == 0)
        //         {
        //             removeAttractor(min_state->attr_id);
        //         }
        //     }
        //     return SUCCESS;
        // }
        if (min_state->state_id == goal_state->state_id || min_state->g == goal_state->g)
        {
            return SUCCESS;
        }

        m_open.pop();
        assert(min_state->g != INFINITECOST);

        expand(min_state);

        expansions += 1;

        m_states[min_state->state_id] = nullptr;
        m_space->DeleteState(min_state->state_id);
        delete min_state;
    }
    printf("Failed to find path\n");
    return EXHAUSTED_OPEN_LIST;
}

void ACLS::extractPath(
    SearchState *to_state,
    std::vector<int> &solution,
    int &cost)
{
    int curr_attr_id = to_state->attr_id;
    int curr_state_id = to_state->state_id;
    int best_pred_state_id;

    solution.push_back(to_state->state_id);
    cost = to_state->g;

    while (!m_space->sameState(curr_state_id, m_start_state_id))
    {

        while (!m_space->sameState(curr_state_id, curr_attr_id))
        {
            best_pred_state_id = m_space->GetBestPredWithCreate(curr_state_id, curr_attr_id);
            SearchState *state;

            state = getSearchState(best_pred_state_id);
            solution.push_back(best_pred_state_id);
            curr_state_id = best_pred_state_id;
        }
        curr_attr_id = m_attractors[curr_attr_id]->pred_attr_id;
    }
    std::reverse(solution.begin(), solution.end());
}

int ACLS::plan(int startID, int goalID, std::vector<int> *solution, int *cost)
{
    m_start_state_id = startID;
    m_goal_state_id = goalID;

    SearchState *start_state = getSearchState(m_start_state_id);
    SearchState *goal_state = getSearchState(m_goal_state_id);

    initSearchState(start_state);
    initSearchState(goal_state);
    start_state->attr_id = startID;
    Attractor *init_attr = createAttractor(start_state);
    init_attr->counter = 1;
    m_space->createAttractor(-1, start_state->state_id);
    m_attractors.resize(2, nullptr);
    m_attractors[startID] = init_attr;

    start_state->g = 0;
    start_state->f = computeFval(start_state);
    start_state->attr_id = startID;

    m_open.push(start_state);

    int err = getPath(goal_state);
    if (err != SUCCESS)
    {
        return err;
    }

    extractPath(goal_state, *solution, *cost);

    return SUCCESS;
}
