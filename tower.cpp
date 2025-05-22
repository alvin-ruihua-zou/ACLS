#include <cmath>
#include <iostream>
#include <cstring>
#include "tower.h"
#include "ACLS.h"
#include <boost/algorithm/string.hpp>

auto std::hash<TowerState>::operator()(
    const argument_type &s) const -> result_type
{
    std::size_t seed = 0;
    int mult = 1;
    for (vector<int> peg : s.pegs)
    {
        for (int disc : peg)
        {
            boost::hash_combine(seed, disc * mult);
        }
        mult = mult * 10;
    }

    return seed;
}

TowerEnvironment::~TowerEnvironment()
{
    DeleteState(m_goal_state_id);
    for (auto *s : m_states)
    {
        if (s != NULL && s != nullptr)
        {
            delete s;
        }
    }
    for (TowerState *a : m_attractors)
    {
        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, TowerState *> p : m_frontier_midpoints)
    {
        TowerState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, TowerState *> p : m_temp_frontier_midpoints)
    {
        TowerState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (TowerState *s : m_ssp)
    {
        delete s;
    }
    m_states.clear();
    m_state_to_id.clear();
    m_frontier_midpoints.clear();
    m_temp_frontier_midpoints.clear();
    m_ssp.clear();
}

TowerState *TowerEnvironment::getHashEntry(int state_id) const
{
    return m_states[state_id];
}

void TowerEnvironment::printState(int state_id)
{

    TowerState *state = getHashEntry(state_id);
    if (state != NULL)
    {
        for (int peg = 0; peg < peg_num; peg++)
        {
            printf("peg %d: ", peg);
            for (int disc : state->pegs[peg])
            {
                printf("%d ", disc);
            }
            printf("\n");
        }
        return;
    }

    state = m_frontier_midpoints[state_id];
    if (state == NULL)
    {
        printf("State %d doesn't exist\n", state_id);
    }
    else
    {
        printf("midpoint:\n");
        for (int peg = 0; peg < peg_num; peg++)
        {
            printf("peg %d: ", peg);
            for (int disc : state->pegs[peg])
            {
                printf("%d ", disc);
            }
            printf("\n");
        }
    }
}

void TowerEnvironment::printAttractors()
{
    printf("---------Printing attractors-----------\n");
    GridState *state;
    auto itr = m_attractors.begin();
    while (itr != m_attractors.end())
    {
        state = *itr;
        if (state != NULL)
        {
            for (int peg = 0; peg < peg_num; peg++)
            {
                printf("peg %d: ", peg);
                for (int disc : state->pegs[peg])
                {
                    printf("%d ", disc);
                }
                printf("\n");
            }
        }
        itr++;
    }
    printf("------------Printing complete-------------\n");
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int TowerEnvironment::getHashEntry(const TowerPegs &pegs)
{
    TowerState state;
    state.pegs = pegs;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end())
    {
        return -1;
    }
    return sit->second;
}

int TowerEnvironment::createHashEntry(
    const TowerPegs &pegs)
{
    int state_id = reserveHashEntry();
    TowerState *entry = getHashEntry(state_id);

    entry->pegs = pegs;
    entry->bp_pegs = pegs;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int TowerEnvironment::getOrCreateState(
    const TowerPegs &pegs)
{
    int state_id = getHashEntry(pegs);
    if (state_id < 0)
    {
        state_id = createHashEntry(pegs);
    }
    return state_id;
}

int TowerEnvironment::reserveHashEntry()
{
    TowerState *entry = new TowerState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}

void TowerEnvironment::setGoalState(const TowerPegs &pegs)
{
    if (m_goal_state_id == -1)
    {
        m_goal_state_id = createHashEntry(pegs);
    }
    m_states[m_goal_state_id]->pegs = pegs;
}

void TowerEnvironment::setStartState(const TowerPegs &pegs)
{
    if (m_start_state_id == -1)
    {
        m_start_state_id = createHashEntry(pegs);
    }
    m_states[m_start_state_id]->pegs = pegs;
}

int TowerEnvironment::getStartStateID()
{
    return m_start_state_id;
}

int TowerEnvironment::getGoalStateID()
{
    return m_goal_state_id;
}

TowerPegs TowerEnvironment::stringToPegs(StateString pegs_string)
{
    int curr_disc;
    int curr_peg = 0;
    TowerPegs pegs(peg_num);

    string delimiter = "_";

    vector<string> tokens;
    boost::split(tokens, pegs_string, boost::is_any_of(delimiter));

    for (const auto &token : tokens)
    {
        curr_disc = stoi(token.c_str());
        if (curr_disc == 0)
        {
            curr_peg += 1;
        }
        else
        {
            pegs[curr_peg].push_back(curr_disc);
        }
    }

    return pegs;
}

string TowerEnvironment::pegsToString(TowerPegs pegs)
{
    int curr_disc;
    int curr_peg;
    StateString pegs_string = "";
    for (vector<int> peg : pegs)
    {
        for (int disc : peg)
        {
            pegs_string += to_string(disc) + "_";
        }
        pegs_string += "0_";
    }
    pegs_string.pop_back();
    return pegs_string;
}

void TowerEnvironment::createAttractor(int state_id, int attr_id)
{
    TowerState *attractor = new TowerState;
    if (m_planner == "ACLS" || m_planner == "FA*" || m_planner == "A*")
    {
        attractor->pegs = m_states[attr_id]->pegs;
    }
    else
    {
        attractor->pegs = m_states[state_id]->bp_pegs;
    }
    if (m_attractors.size() <= attr_id)
    {
        m_attractors.resize(attr_id + 1, nullptr);
    }
    m_attractors[attr_id] = attractor;
}

void TowerEnvironment::removeAttractor(int state_id)
{
    delete m_attractors[state_id];
    m_attractors[state_id] = nullptr;
}

double TowerEnvironment::GetAttractorDist(int state_id, int attr_id)
{
    TowerPegs pegs = m_states[state_id]->pegs;
    return GetAttractorDist(pegs, attr_id);
}

double TowerEnvironment::GetAttractorDist(const TowerPegs &pegs, int attr_id)
{
    TowerState *attr_state = m_attractors[attr_id];
    int differenceCount = 0;

    // Loop through both outer vectors
    for (size_t i = 0; i < attr_state->pegs.size(); ++i)
    {
        // Get the sizes of the current inner vectors
        size_t size1 = attr_state->pegs[i].size();
        size_t size2 = pegs[i].size();

        // Determine the maximum size to compare elements, handling different lengths
        size_t maxSize = std::max(size1, size2);

        // Compare elements in both inner vectors
        for (size_t j = 0; j < maxSize; ++j)
        {
            // Check if either vector is shorter at this position
            int val1 = (j < size1) ? attr_state->pegs[i][j] : -1; // Use -1 for out-of-bounds
            int val2 = (j < size2) ? pegs[i][j] : -1;             // Use -1 for out-of-bounds

            // If the values differ, increment the difference count
            if (val1 != val2)
            {
                differenceCount++;
            }
        }
    }

    return differenceCount;
}

double TowerEnvironment::GetGoalHeuristic(int state_id)
{
    TowerState *goal_state = m_states[m_goal_state_id];
    TowerPegs pegs = m_states[state_id]->pegs;
    int differenceCount = 0;

    // Loop through both outer vectors
    for (size_t i = 0; i < goal_state->pegs.size(); ++i)
    {
        // Get the sizes of the current inner vectors
        size_t size1 = goal_state->pegs[i].size();
        size_t size2 = pegs[i].size();

        // Determine the maximum size to compare elements, handling different lengths
        size_t maxSize = std::max(size1, size2);

        // Compare elements in both inner vectors
        for (size_t j = 0; j < maxSize; ++j)
        {
            // Check if either vector is shorter at this position
            int val1 = (j < size1) ? goal_state->pegs[i][j] : -1; // Use -1 for out-of-bounds
            int val2 = (j < size2) ? pegs[i][j] : -1;             // Use -1 for out-of-bounds

            // If the values differ, increment the difference count
            if (val1 != val2)
            {
                differenceCount++;
            }
        }
    }

    return differenceCount;
}

void TowerEnvironment::GetSuccs(int state_id, bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    TowerState *state = m_states[state_id];
    TowerState *succ_state;
    int succ_state_id;
    int moving_disc;
    int disc_to_be_stacked_on;
    TowerPegs new_pegs;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        if (!action_operators.test(idx))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        new_pegs = state->pegs;
        pair<int, int> action = m_actions[idx];
        if (state->pegs[action.first].empty())
        {
            continue;
        }
        moving_disc = state->pegs[action.first].back();
        if (state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = state->pegs[action.second].back();
        }

        if (moving_disc > disc_to_be_stacked_on)
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        new_pegs[action.second].push_back(moving_disc);
        new_pegs[action.first].pop_back();

        succ_state_id = getOrCreateState(new_pegs);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

void TowerEnvironment::GetSuccsAndBestPred(int state_id, bitset<NUM_OF_ACTIONS> action_operators, int attr_id, int bp_idx, bool *best_pred_match, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    TowerState *state = m_states[state_id];
    TowerState *succ_state;
    int succ_state_id;
    int moving_disc;
    int disc_to_be_stacked_on;
    TowerPegs new_pegs;
    TowerPegs best_pred_pegs;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        new_pegs = state->pegs;
        pair<int, int> action = m_actions[idx];
        if (state->pegs[action.first].empty())
        {
            continue;
        }
        moving_disc = state->pegs[action.first].back();
        if (state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = state->pegs[action.second].back();
        }

        if (moving_disc > disc_to_be_stacked_on)
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        new_pegs[action.second].push_back(moving_disc);
        new_pegs[action.first].pop_back();

        curr_dist = GetAttractorDist(new_pegs, attr_id);
        if (curr_dist < best_pred_dist)
        {
            best_pred_pegs = new_pegs;
            best_pred_dist = curr_dist;
        }

        if (!action_operators.test(idx))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }

        succ_state_id = getOrCreateState(new_pegs);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
    if (best_pred_pegs == state->bp_pegs)
    {
        *best_pred_match = true;
    }
    else
    {
        *best_pred_match = false;
    }
}

pair<int, int> TowerEnvironment::recoverAction(const TowerPegs &pegs1, const TowerPegs &pegs2)
{
    int source = -1, destination = -1;

    for (int i = 0; i < pegs1.size(); ++i)
    {
        // Check if the peg size has decreased
        if (pegs1[i].size() > pegs2[i].size())
        {
            source = i;
        }
        // Check if the peg size has increased
        else if (pegs1[i].size() < pegs2[i].size())
        {
            destination = i;
        }
    }

    return {source, destination};
}

int TowerEnvironment::getMatchingState(int state_id, int attr_id, int bp_id, int bp2_id, int bp2_attr_id)
{

    TowerState *state = m_states[state_id];
    TowerState *bp2_state = m_states[bp2_id];

    if (GetAttractorDist(state->bp_pegs, attr_id) > GetAttractorDist(bp2_id, bp2_attr_id))
    {
        return bp_id;
    }
    else if (GetAttractorDist(state->bp_pegs, attr_id) < GetAttractorDist(bp2_id, bp2_attr_id))
    {
        return bp2_id;
    }

    pair<int, int> action1 = recoverAction(state->bp_pegs, state->pegs);
    pair<int, int> action2 = recoverAction(bp2_state->pegs, state->pegs);
    for (pair<int, int> action : m_actions)
    {
        if (action == action1)
        {
            return bp_id;
        }
        if (action == action2)
        {
            return bp2_id;
        }
    }
    return -1;
}

void TowerEnvironment::GetSuccs(int state_id, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    TowerState *state = m_states[state_id];
    TowerState *succ_state;
    int succ_state_id;
    int moving_disc;
    int disc_to_be_stacked_on;
    TowerPegs new_pegs;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        new_pegs = state->pegs;
        pair<int, int> action = m_actions[idx];
        if (state->pegs[action.first].empty())
        {
            continue;
        }
        moving_disc = state->pegs[action.first].back();
        if (state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = state->pegs[action.second].back();
        }

        if (moving_disc > disc_to_be_stacked_on)
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        new_pegs[action.second].push_back(moving_disc);
        new_pegs[action.first].pop_back();

        succ_state_id = getOrCreateState(new_pegs);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

int TowerEnvironment::GetBestPred(int state_id, int attr_id)
{
    TowerState *state = m_states[state_id];
    int moving_disc;
    int disc_to_be_stacked_on;
    TowerPegs pred_pegs;
    int best_pred_state_id;
    TowerPegs best_pred_pegs = state->pegs;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        pred_pegs = state->pegs;
        pair<int, int> action = m_actions[idx];
        if (state->pegs[action.first].empty())
        {
            continue;
        }
        moving_disc = state->pegs[action.first].back();
        if (state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = state->pegs[action.second].back();
        }

        if (moving_disc > disc_to_be_stacked_on)
        {
            continue;
        }
        pred_pegs[action.second].push_back(moving_disc);
        pred_pegs[action.first].pop_back();

        curr_dist = GetAttractorDist(pred_pegs, attr_id);

        if (curr_dist < best_pred_dist)
        {
            best_pred_pegs = pred_pegs;
            best_pred_dist = curr_dist;
        }
    }
    // Returns -1 if the pred state is not yet created
    best_pred_state_id = getHashEntry(best_pred_pegs);
    return best_pred_state_id;
}

int TowerEnvironment::GetBestPredWithCreate(int state_id, int attr_id)
{
    TowerState *state = m_states[state_id];
    int moving_disc;
    int disc_to_be_stacked_on;
    TowerPegs pred_pegs;
    int best_pred_state_id;
    TowerPegs best_pred_pegs = state->pegs;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        pred_pegs = state->pegs;
        pair<int, int> action = m_actions[idx];
        if (state->pegs[action.first].empty())
        {
            continue;
        }
        moving_disc = state->pegs[action.first].back();
        if (state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = state->pegs[action.second].back();
        }

        if (moving_disc > disc_to_be_stacked_on)
        {
            continue;
        }
        pred_pegs[action.second].push_back(moving_disc);
        pred_pegs[action.first].pop_back();

        curr_dist = GetAttractorDist(pred_pegs, attr_id);
        if (curr_dist < best_pred_dist)
        {
            best_pred_pegs = pred_pegs;
            best_pred_dist = curr_dist;
        }
    }

    best_pred_state_id = getHashEntry(best_pred_pegs);
    if (best_pred_state_id < 0)
    {
        best_pred_state_id = createHashEntry(best_pred_pegs);
    }
    return best_pred_state_id;
}

int TowerEnvironment::getInDegree(int state_id)
{
    TowerState *state = m_states[state_id];
    int p = 0;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        TowerPegs new_pegs = state->pegs;
        pair<int, int> action = m_actions[idx];
        if (state->pegs[action.first].empty())
        {
            continue;
        }
        int moving_disc = state->pegs[action.first].back();
        int disc_to_be_stacked_on;
        if (state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = state->pegs[action.second].back();
        }

        if (moving_disc <= disc_to_be_stacked_on)
        {
            p++;
        }
    }
    return p;
}

void TowerEnvironment::addSspState(int state_id)
{
    TowerState *s = new TowerState;
    s->pegs = m_states[state_id]->pegs;
    m_ssp.push_back(s);
}

bool TowerEnvironment::sameState(int state_id, int attr_id)
{
    TowerState *state = getHashEntry(state_id);
    TowerState *attractor = m_attractors[attr_id];
    if (state == NULL || attractor == NULL)
    {
        return false;
    }
    return state->pegs == attractor->pegs;
}

void TowerEnvironment::DeleteState(int state_id)
{

    if (m_states[state_id] == NULL)
    {
        return;
    }
    m_state_to_id.erase(m_states[state_id]);
    delete m_states[state_id];
    m_states[state_id] = nullptr;
}

bool TowerEnvironment::isNeighbor(string search, int start_state_id, int goal_state_id)
{
    TowerState *start_state, *goal_state;
    if (search == "frontier")
    {
        start_state = m_frontier_midpoints[start_state_id];
        goal_state = m_frontier_midpoints[goal_state_id];
    }
    else if (search == "smgs")
    {
        start_state = m_ssp[start_state_id];
        goal_state = m_ssp[goal_state_id];
    }
    else
    {
        start_state = getHashEntry(start_state_id);
        goal_state = getHashEntry(goal_state_id);
    }

    TowerPegs new_pegs;
    int moving_disc;
    int disc_to_be_stacked_on;
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        new_pegs = start_state->pegs;
        pair<int, int> action = m_actions[idx];
        if (start_state->pegs[action.first].empty())
        {
            continue;
        }
        moving_disc = start_state->pegs[action.first].back();
        if (start_state->pegs[action.second].empty())
        {
            disc_to_be_stacked_on = 100000;
        }
        else
        {
            disc_to_be_stacked_on = start_state->pegs[action.second].back();
        }

        if (moving_disc > disc_to_be_stacked_on)
        {
            continue;
        }
        new_pegs[action.second].push_back(moving_disc);
        new_pegs[action.first].pop_back();
        if (new_pegs == goal_state->pegs)
        {
            return true;
        }
    }

    return false;
}

void TowerEnvironment::UpdateBP(int state_id, int bp_state_id)
{
    TowerState *s = m_states[state_id];
    TowerState *bp = m_states[bp_state_id];
    s->bp_pegs = bp->pegs;
}

void TowerEnvironment::AddMidpointState(int stateID, int newID)
{
    TowerState *s = new TowerState;
    if (m_temp_frontier_midpoints.empty())
    {
        s->pegs = m_states[stateID]->pegs;
    }
    else
    {
        s->pegs = m_temp_frontier_midpoints[stateID]->pegs;
    }

    m_frontier_midpoints[newID] = s;
}

void TowerEnvironment::AddTempMidpointState(int stateID)
{

    TowerState *s = new TowerState;
    s->pegs = m_states[stateID]->pegs;
    m_temp_frontier_midpoints[stateID] = s;
}

void TowerEnvironment::ClearTempMidpoints()
{

    for (auto p : m_temp_frontier_midpoints)
    {
        TowerState *s = p.second;
        delete s;
    }
    m_temp_frontier_midpoints.clear();
}

void TowerEnvironment::reinit_frontier_search(int startStateID, int goalStateID)
{
    // To ensure consistency between planner and environment
    m_states.clear();
    m_state_to_id.clear();
    m_states.resize(max(startStateID, goalStateID) + 1, nullptr);
    TowerState *start = new TowerState;
    start->pegs = m_frontier_midpoints[startStateID]->pegs;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    TowerState *goal = new TowerState;
    goal->pegs = m_frontier_midpoints[goalStateID]->pegs;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

void TowerEnvironment::reinit_smgs(int startStateID, int goalStateID, int start_ssp_idx, int goal_ssp_idx)
{
    for (TowerState *s : m_states)
    {
        if (s != NULL)
        {
            m_state_to_id.erase(s);
            delete s;
        }
    }
    m_states.clear();
    m_state_to_id.clear();
    m_states.resize(max(startStateID, goalStateID) + 1, nullptr);
    TowerState *start = new TowerState;
    start->pegs = m_ssp[start_ssp_idx]->pegs;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    TowerState *goal = new TowerState;
    goal->pegs = m_ssp[goal_ssp_idx]->pegs;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

bool TowerEnvironment::init(int peg_disc_num, int peg_peg_num, vector<pair<int, int>> actions, vector<double> costs, string planner)
{
    if (m_goal_state_id != -1)
    {
        DeleteState(m_goal_state_id);
    }
    for (auto *s : m_states)
    {
        if (s != NULL && s != nullptr)
        {
            delete s;
        }
    }
    for (TowerState *a : m_attractors)
    {

        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, TowerState *> p : m_frontier_midpoints)
    {
        TowerState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (TowerState *s : m_ssp)
    {
        delete s;
    }
    m_states.clear();
    m_state_to_id.clear();
    m_attractors.clear();
    m_temp_frontier_midpoints.clear();
    m_frontier_midpoints.clear();
    m_ssp.clear();
    m_start_state_id = -1;
    m_goal_state_id = -1;
    m_weight = 1;
    m_planner = planner;
    disc_num = peg_disc_num;
    peg_num = peg_peg_num;
    m_actions = actions;
    m_costs = costs;
    return true;
}

bool TowerEnvironment::extractPath(
    string search,
    const std::vector<int> &idpath,
    std::vector<TowerState *> &path)
{
    if (idpath.empty())
    {
        return true;
    }

    // grab the rest of the points
    for (size_t i = 0; i < idpath.size(); ++i)
    {
        auto curr_id = idpath[i];
        auto *entry = getHashEntry(idpath[0]);
        if (search == "frontier")
        {
            entry = m_frontier_midpoints[idpath[0]];
        }
        if (!entry)
        {
            return false;
        }
        path.push_back(entry);
    }
    return true;
}
