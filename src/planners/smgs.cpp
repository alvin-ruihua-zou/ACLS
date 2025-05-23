#include <math.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <chrono>
#include "smgs.h"
#include "compares.h"
using namespace std;

SMGS::SMGS(DiscreteSpaceInformation *space)
    : m_space(space),
      m_open(),
      m_states(),
      m_midpoint_states(),
      m_closed(),
      m_ssp(),
      m_start_state_id(-1),
      m_goal_state_id(-1),
      m_weight(1),
      m_closed_threshold(0),
      m_closed_max_threshold(0),
      m_closed_size(0),
      m_closed_max_size(0),
      m_timeout(INFINITECOST),
      m_succs(),
      m_costs()
{
}

SMGS::~SMGS()
{
    for (SearchState *s : m_states)
    {
        if (s != NULL)
        {
            delete s;
        }
    }

    for (SearchState *s : m_ssp)
    {
        if (s != NULL)
        {
            delete s;
        }
    }

    m_states.clear();
    m_ssp.clear();
    m_closed.clear();
}

void SMGS::printState(int state_id)
{
    SearchState *s = m_states[state_id];
    if (s == nullptr || s == NULL)
    {
        printf("relay: ");
        s = m_ssp[state_id];
    }
    if (s == nullptr || s == NULL)
    {
        printf("invalid state: %d\n", state_id);
        return;
    }
    printf("StateID: %d, g: %f, h: %f, p: %d, ", s->state_id, s->g, s->h, s->p);
    if (s->ancestor == NULL)
    {
        printf("ancestor: NULL\n");
    }
    else
    {
        printf("ancestor: %d\n", s->ancestor->state_id);
    }
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
SMGS::SearchState *SMGS::getSearchState(int state_id)
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
SMGS::SearchState *SMGS::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState *ss = new SearchState;
    ss->state_id = state_id;
    initSearchState(ss);

    return ss;
}

// Initialize a search state.
void SMGS::initSearchState(SearchState *state)
{

    state->g = INFINITECOST;
    state->h = m_space->GetGoalHeuristic(state->state_id);
    // state->f = computeFval(state);

    state->p = m_space->getInDegree(state->state_id);

    state->ancestor = nullptr;
}

double SMGS::computeFval(SearchState *s) const
{
    return s->g + s->h * m_weight;
}

bool SMGS::almostEqual(double c1, double c2)
{
    if (abs(c1 - c2) < 1e-6)
    {
        return true;
    }
    return false;
}

enum PlanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    EXHAUSTED_OPEN_LIST,
    TIMEOUT
};

void SMGS::pruneClosedList()
{
    SearchState *s;
    vector<int> succs;
    vector<double> costs;
    vector<int> action_ids;
    for (auto itr = m_open.begin(); itr != m_open.end(); itr++)
    {
        s = *itr;
        succs.clear();
        costs.clear();
        action_ids.clear();
        m_space->GetSuccs(s->state_id, &succs, &costs, &action_ids);
        bool is_pred = false;
        for (size_t idx = 0; idx < succs.size(); ++idx)
        {
            int succ_state_id = succs[idx];
            int cost = costs[idx];
            int action_id = action_ids[idx];

            if (s->ancestor != NULL && s->ancestor->state_id == succ_state_id)
            {
                is_pred = true;
                break;
            }
        }
        if (is_pred)
        {
            SearchState *a = s->ancestor;
            auto closed_itr = m_closed.find(a->state_id);
            if (closed_itr != NULL) // Closed
            {
                while (a->p == 0)
                {
                    a = a->ancestor;
                    closed_itr = m_closed.find(a->state_id);
                    if (closed_itr == NULL)
                    {
                        break;
                    }
                }
            }
            if (a->state_id != s->ancestor->state_id)
            {
                s->ancestor = a;
                a->p = INFINITECOST;
            }
        }
    }

    for (auto itr = m_closed.begin(); itr != m_closed.end(); itr++)
    {
        s = itr->second;
        if (s == NULL)
        {
            continue;
        }
        succs.clear();
        costs.clear();
        action_ids.clear();
        m_space->GetSuccs(s->state_id, &succs, &costs, &action_ids);
        bool is_pred = false;
        for (size_t idx = 0; idx < succs.size(); ++idx)
        {
            int succ_state_id = succs[idx];
            int cost = costs[idx];
            int action_id = action_ids[idx];

            if (s->ancestor != NULL && s->ancestor->state_id == succ_state_id)
            {
                is_pred = true;
                break;
            }
        }
        if (is_pred)
        {
            SearchState *a = s->ancestor;
            auto closed_itr = m_closed.find(a->state_id);
            if (closed_itr != NULL) // Closed
            {
                while (a->p == 0)
                {
                    a = a->ancestor;
                    closed_itr = m_closed.find(a->state_id);
                    if (closed_itr == NULL)
                    {
                        break;
                    }
                }
            }
            if (a->state_id != s->ancestor->state_id)
            {
                s->ancestor = a;
                a->p = INFINITECOST;
            }
        }
    }

    for (pair<int, SearchState *> s : m_closed)
    {
        if (s.second == NULL)
        {
            continue;
        }
        if (s.second->p == 0)
        {
            m_space->DeleteState(s.first);
            delete m_closed[s.first];
            m_closed[s.first] = nullptr;
            m_states[s.first] = nullptr;
            m_closed_size--;
        }
    }
}
// struct SearchState : public heap_element;
vector<SMGS::SearchState *> SMGS::extractSparseSolution(SearchState *s)
{
    vector<SearchState *> path;
    SearchState *a = s->ancestor;
    path.push_back(s);
    while (a != m_states[m_start_state_id])
    {
        path.push_back(a);
        a = a->ancestor;
    }
    path.push_back(a);
    std::reverse(path.begin(), path.end());
    return path;
}

void SMGS::extractPath(
    SearchState *start_state,
    SearchState *goal_state,
    std::vector<int> &solution)
{

    m_start_state_id = start_state->state_id;
    m_goal_state_id = goal_state->state_id;
    SearchState *search_goal_state = getSearchState(m_goal_state_id);
    SearchState *search_start_state = getSearchState(m_start_state_id);

    search_start_state->g = 0;
    search_start_state->f = computeFval(search_start_state);
    search_start_state->p = INFINITECOST;

    m_open.push(search_start_state);
    m_closed_size = 0;

    while (!m_open.empty())
    {
        SearchState *min_state = m_open.min();

        m_open.pop();
        m_closed.insert(make_pair(min_state->state_id, min_state));
        m_closed_size++;

        if (m_closed_size > m_closed_max_size)
        {
            m_closed_max_size = m_closed_size;
        }

        // path to goal found
        if (min_state->state_id == search_goal_state->state_id || min_state->g == search_goal_state->g)
        {
            vector<SearchState *> path = extractSparseSolution(search_goal_state);

            SearchState *s0 = new SearchState;
            s0->f = path[0]->f;
            s0->g = path[0]->g;
            s0->h = path[0]->h;
            s0->ancestor = s0;
            s0->p = path[0]->p;
            s0->state_id = path[0]->state_id;
            int start_idx = m_ssp.size() - 1;

            m_ssp.push_back(s0);
            m_space->addSspState(s0->state_id);
            start_idx++;

            for (int i = 1; i < path.size(); i++)
            {
                SearchState *s = new SearchState;
                s->f = path[i]->f;
                s->g = path[i]->g;
                s->h = path[i]->h;
                s->ancestor = m_ssp[start_idx + i - 1];
                s->p = path[i]->p;
                s->state_id = path[i]->state_id;

                m_ssp.push_back(s);
                m_space->addSspState(s->state_id);
            }

            int ssp_size = m_ssp.size();
            for (int i = start_idx + 1; i < ssp_size; i++)
            {

                if (m_space->isNeighbor("smgs", i - 1, i))
                {
                    solution.push_back((m_ssp[i]->state_id));
                }
                else
                {
                    while (!m_open.empty())
                    {
                        min_state = m_open.min();
                        m_open.pop();
                    }
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

                    m_closed.clear();

                    m_open.clear();
                    m_space->reinit_smgs(m_ssp[i - 1]->state_id, m_ssp[i]->state_id, i - 1, i);
                    extractPath(m_ssp[i - 1], m_ssp[i], solution);
                }
            }
            return;
        }

        // Expand
        m_succs.clear();
        m_costs.clear();
        m_action_ids.clear();
        SearchState *s = min_state;

        // Assign in degree
        if (s->state_id != search_start_state->state_id)
        {
            s->p = m_space->getInDegree(s->state_id);
        }

        m_space->GetSuccs(s->state_id, &m_succs, &m_costs, &m_action_ids);
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
            auto itr = m_closed.find(succ_state->state_id);
            if (itr != NULL) // Closed
            {
                succ_state->p -= 1;
                s->p -= 1;
                continue;
            }

            int new_cost = s->g + cost;
            // If succ_state not in open its cost is INFINITY
            // So condition checks (if succ_state is in OPEN and better cost) or (not in open)
            if (new_cost < succ_state->g)
            {
                bool in_open = false;
                if (succ_state->g != INFINITECOST)
                {
                    in_open = true;
                }
                succ_state->g = new_cost;
                succ_state->f = computeFval(succ_state);
                succ_state->ancestor = s;
                if (in_open)
                {
                    m_open.decrease(succ_state);
                }
                else if (itr == NULL) // Only add to OPEN if state isn't closed
                {
                    m_open.push(succ_state);

                    if (m_closed_size > m_closed_threshold && m_closed_size < m_closed_max_threshold)
                    {
                        t2 = chrono::high_resolution_clock::now();
                        if (chrono::duration_cast<chrono::duration<double>>(t2 - t1).count() > m_timeout)
                        {
                            solution = {};
                            return;
                        }
                        pruneClosedList();
                    }
                }
            }
        }
    }

    return;
}

int SMGS::plan(int startID, int goalID, vector<int> *solution, int *cost)
{
    m_start_state_id = startID;
    m_goal_state_id = goalID;

    SearchState *start_state = new SearchState;
    start_state->state_id = m_start_state_id;

    SearchState *goal_state = new SearchState;
    goal_state->state_id = m_goal_state_id;

    printf("closed threshold: %d\n", m_closed_threshold);
    t1 = chrono::high_resolution_clock::now();
    extractPath(start_state, goal_state, *solution);

    delete start_state;
    delete goal_state;

    return SUCCESS;
}

int SMGS::getClosedListSize()
{
    int size = 0;
    for (pair<int, SearchState *> s : m_closed)
    {
        if (s.second != NULL)
        {
            size++;
        }
    }
    return size;
}