#include <cmath>
#include <iostream>
#include <cstring>
#include "2dgrid.h"
#include "compares.h"
#include "ACLS.h"

using namespace Grid2D;

GridEnvironment::~GridEnvironment()
{
    DeleteState(m_goal_state_id);
    for (auto *s : m_states)
    {
        if (s != NULL && s != nullptr)
        {
            delete s;
        }
    }

    for (GridState *a : m_attractors)
    {
        if (a != NULL)
        {
            delete a;
        }
    }

    for (pair<const int, GridState *> p : m_frontier_midpoints)
    {
        GridState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, GridState *> p : m_temp_frontier_midpoints)
    {
        GridState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }

    for (GridState *s : m_ssp)
    {
        delete s;
    }

    m_states.clear();
    m_state_to_id.clear();
    m_frontier_midpoints.clear();
    m_temp_frontier_midpoints.clear();
    m_ssp.clear();
}

GridState *GridEnvironment::getHashEntry(int state_id) const
{
    return m_states[state_id];
}

void GridEnvironment::printState(int state_id)
{

    GridState *state = getHashEntry(state_id);
    if (state != NULL)
    {
        printf("State %d: coord %d, %d\n", state_id, state->coord.first, state->coord.second);
        state = m_frontier_midpoints[state_id];
        if (state != NULL)

        {
            printf("midpoint: State %d: coord %d, %d\n", state_id, state->coord.first, state->coord.second);
        }
        return;
    }

    state = m_frontier_midpoints[state_id];
    if (state != NULL)
    {
        printf("midpoint: State %d: coord %d, %d\n", state_id, state->coord.first, state->coord.second);
        return;
    }

    state = m_temp_frontier_midpoints[state_id];
    if (state == NULL)
    {
        printf("State %d doesn't exist\n", state_id);
    }
    else
    {
        printf("temp midpoint: State %d: coord %d, %d\n", state_id, state->coord.first, state->coord.second);
    }
}

void GridEnvironment::printAttractors()
{
    printf("---------Printing attractors-----------\n");
    GridState *state;
    auto itr = m_attractors.begin();
    int i = 0;
    while (itr != m_attractors.end())
    {
        if ((*itr) != NULL)
        {
            printf("Key: %d Coord: %d, %d\n", i, (*itr)->coord.first, (*itr)->coord.second);
        }
        itr++;
        i++;
    }
    printf("------------Printing complete-------------\n");
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int GridEnvironment::getHashEntry(const RobotCoord &coord)
{
    GridState state;
    state.coord = coord;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end())
    {
        return -1;
    }
    return sit->second;
}

int GridEnvironment::createHashEntry(
    const RobotCoord &coord)
{
    int state_id = reserveHashEntry();
    GridState *entry = getHashEntry(state_id);

    entry->coord = coord;
    entry->bp_coord = coord;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int GridEnvironment::getOrCreateState(
    const RobotCoord &coord)
{
    int state_id = getHashEntry(coord);
    if (state_id < 0)
    {
        state_id = createHashEntry(coord);
    }
    return state_id;
}

int GridEnvironment::reserveHashEntry()
{
    GridState *entry = new GridState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}

void GridEnvironment::setGoalState(const RobotCoord &coord)
{
    if (m_goal_state_id == -1)
    {
        m_goal_state_id = createHashEntry(coord);
    }
    m_states[m_goal_state_id]->coord = coord;
}

void GridEnvironment::setStartState(const RobotCoord &coord)
{
    if (m_start_state_id == -1)
    {
        m_start_state_id = createHashEntry(coord);
    }
    m_states[m_start_state_id]->coord = coord;
}

int GridEnvironment::getStartStateID()
{
    return m_start_state_id;
}

int GridEnvironment::getGoalStateID()
{
    return m_goal_state_id;
}

bool GridEnvironment::coordFree(const RobotCoord &coord)
{
    int row = coord.first;
    int col = coord.second;
    if (col >= 0 && col < map_col_size && row >= 0 && row < map_row_size &&
        stateMap[row][col] == 0)
    {
        return true;
    }
    return false;
}

void GridEnvironment::createAttractor(int state_id, int attr_id)
{
    GridState *attractor = new GridState;

    if (m_planner == "ACLS" || m_planner == "FA*" || m_planner == "A*")
    {
        attractor->coord = m_states[attr_id]->coord;
    }
    else
    {
        attractor->coord = m_states[state_id]->bp_coord;
    }
    if (m_attractors.size() <= attr_id)
    {
        m_attractors.resize(attr_id + 1, nullptr);
    }
    m_attractors[attr_id] = attractor;
}

void GridEnvironment::removeAttractor(int state_id)
{
    delete m_attractors[state_id];
    m_attractors[state_id] = nullptr;
}

double GridEnvironment::GetAttractorDist(int state_id, int attr_id)
{
    RobotCoord coord = m_states[state_id]->coord;
    return GetAttractorDist(coord, attr_id);
}

double GridEnvironment::GetAttractorDist(const RobotCoord &coord, int attr_id)
{
    GridState *attr_state = m_attractors[attr_id];
    return m_weight * (abs(coord.first - attr_state->coord.first) + abs(coord.second - attr_state->coord.second));
    // return m_weight * hypot(coord.first - attr_state->coord.first, coord.second - attr_state->coord.second);
}

double GridEnvironment::GetGoalHeuristic(int state_id)
{
    RobotCoord goal_state_coord = m_states[m_goal_state_id]->coord;
    RobotCoord state_coord = m_states[state_id]->coord;
    return abs(state_coord.first - goal_state_coord.first) + abs(state_coord.second - goal_state_coord.second);
    // return hypot(state_coord.first - goal_state_coord.first, state_coord.second - goal_state_coord.second);
}

void GridEnvironment::GetSuccs(int state_id, bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    GridState *state = m_states[state_id];
    GridState *succ_state;
    int succ_state_id;
    RobotCoord succ_coord;

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
        succ_coord = state->coord + m_actions[idx];

        if (!coordFree(succ_coord))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        succ_state_id = getOrCreateState(succ_coord);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

void GridEnvironment::GetSuccsAndBestPred(int state_id, bitset<NUM_OF_ACTIONS> action_operators, int attr_id, int bp_idx, bool *best_pred_match, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    GridState *state = m_states[state_id];
    GridState *succ_state;
    int succ_state_id;
    RobotCoord succ_coord;
    RobotCoord pred_coord;
    RobotCoord best_pred_coord = state->coord;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        succ_coord = state->coord + m_actions[idx];
        pred_coord = succ_coord;
        if (!coordFree(succ_coord))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }

        // Calculating best pred
        // GridState *attr_state = m_attractors[attr_id];
        // curr_dist = m_weight * (abs(pred_coord.first - attr_state->coord.first) + abs(pred_coord.second - attr_state->coord.second));
        curr_dist = GetAttractorDist(pred_coord, attr_id);

        if (isLessThan(curr_dist, best_pred_dist))
        {
            best_pred_coord = pred_coord;
            best_pred_dist = curr_dist;
        }
        if (!action_operators.test(idx))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);

            continue;
        }

        succ_state_id = getOrCreateState(succ_coord);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
    if (best_pred_coord == state->bp_coord)
    {
        *best_pred_match = true;
    }
    else
    {
        *best_pred_match = false;
    }
}

int GridEnvironment::getMatchingState(int state_id, int attr_id, int bp_id, int bp2_id, int bp2_attr_id)
{
    GridState *state = m_states[state_id];
    GridState *bp2_state = m_states[bp2_id];

    if (isGreaterThan(GetAttractorDist(state->bp_coord, attr_id), GetAttractorDist(bp2_id, bp2_attr_id)))
    {
        return bp_id;
    }
    else if (isLessThan(GetAttractorDist(state->bp_coord, attr_id), GetAttractorDist(bp2_id, bp2_attr_id)))
    {
        return bp2_id;
    }

    pair<int, int> action1 = state->bp_coord - state->coord;
    pair<int, int> action2 = bp2_state->coord - state->coord;
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

void GridEnvironment::GetSuccs(int state_id, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    GridState *state = m_states[state_id];
    GridState *succ_state;
    int succ_state_id;
    RobotCoord succ_coord;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        succ_coord = state->coord + m_actions[idx];

        if (!coordFree(succ_coord))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        succ_state_id = getOrCreateState(succ_coord);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

int GridEnvironment::GetBestPred(int state_id, int attr_id)
{
    GridState *state = getHashEntry(state_id);
    int best_pred_state_id;
    RobotCoord pred_coord;
    RobotCoord best_pred_coord = state->coord;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        pred_coord = state->coord + m_actions[idx];

        if (!coordFree(pred_coord))
        {
            continue;
        }

        // GridState *attr_state = m_attractors[attr_id];
        // curr_dist = m_weight * (abs(pred_coord.first - attr_state->coord.first) + abs(pred_coord.second - attr_state->coord.second));
        curr_dist = GetAttractorDist(pred_coord, attr_id);
        if (curr_dist < best_pred_dist)
        {
            best_pred_coord = pred_coord;
            best_pred_dist = curr_dist;
        }
    }
    // Returns -1 if the pred state is not yet created
    best_pred_state_id = getHashEntry(best_pred_coord);
    return best_pred_state_id;
}

int GridEnvironment::GetBestPredWithCreate(int state_id, int attr_id)
{
    GridState *state = getHashEntry(state_id);
    int best_pred_state_id;
    RobotCoord pred_coord;
    RobotCoord best_pred_coord = state->coord;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        pred_coord = state->coord + m_actions[idx];

        if (!coordFree(pred_coord))
        {
            continue;
        }
        // GridState *attr_state = m_attractors[attr_id];
        // curr_dist = m_weight * (abs(pred_coord.first - attr_state->coord.first) + abs(pred_coord.second - attr_state->coord.second));
        curr_dist = GetAttractorDist(pred_coord, attr_id);

        if (curr_dist < best_pred_dist)
        {
            best_pred_coord = pred_coord;
            best_pred_dist = curr_dist;
        }
    }
    // Returns -1 if the pred state is not yet created
    best_pred_state_id = getHashEntry(best_pred_coord);
    if (best_pred_state_id < 0)
    {
        best_pred_state_id = createHashEntry(best_pred_coord);
    }
    return best_pred_state_id;
}

int GridEnvironment::getInDegree(int state_id)
{
    GridState *state = m_states[state_id];
    int p = 0;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        RobotCoord succ_coord = state->coord + m_actions[idx];

        if (coordFree(succ_coord))
        {
            p++;
        }
    }
    return p;
}

void GridEnvironment::addSspState(int state_id)
{
    GridState *s = new GridState;
    s->coord = m_states[state_id]->coord;
    m_ssp.push_back(s);
}

bool GridEnvironment::sameState(int state_id, int attr_id)
{

    GridState *state = getHashEntry(state_id);
    GridState *attractor = m_attractors[attr_id];

    if (state == NULL || attractor == NULL)
    {
        return false;
    }
    return state->coord == attractor->coord;
}

void GridEnvironment::DeleteState(int state_id)
{

    if (m_states[state_id] == NULL)
    {
        return;
    }
    m_state_to_id.erase(m_states[state_id]);
    delete m_states[state_id];
    m_states[state_id] = nullptr;
}

bool GridEnvironment::isNeighbor(string search, int start_state_id, int goal_state_id)
{
    GridState *start_state, *goal_state;
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

    RobotCoord diff = start_state->coord - goal_state->coord;
    if (abs(diff.first) + abs(diff.second) <= 1)
    {
        return true;
    }
    return false;
}

void GridEnvironment::UpdateBP(int state_id, int bp_state_id)
{
    GridState *s = m_states[state_id];
    GridState *bp = m_states[bp_state_id];
    s->bp_coord = bp->coord;
}

void GridEnvironment::AddMidpointState(int stateID, int newID)
{
    GridState *s = new GridState;
    if (m_temp_frontier_midpoints.empty())
    {
        s->coord = m_states[stateID]->coord;
    }
    else
    {
        s->coord = m_temp_frontier_midpoints[stateID]->coord;
    }

    m_frontier_midpoints[newID] = s;
}

void GridEnvironment::AddTempMidpointState(int stateID)
{
    GridState *s = new GridState;
    s->coord = m_states[stateID]->coord;
    m_temp_frontier_midpoints[stateID] = s;
}

void GridEnvironment::ClearTempMidpoints()
{
    for (auto p : m_temp_frontier_midpoints)
    {
        GridState *s = p.second;
        delete s;
    }
    m_temp_frontier_midpoints.clear();
}

void GridEnvironment::reinit_frontier_search(int startStateID, int goalStateID)
{

    // To ensure consistency between planner and environment
    m_states.clear();
    m_state_to_id.clear();
    m_temp_frontier_midpoints.clear();
    m_states.resize(max(startStateID, goalStateID) + 1, nullptr);
    GridState *start = new GridState;
    start->coord = m_frontier_midpoints[startStateID]->coord;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    GridState *goal = new GridState;
    goal->coord = m_frontier_midpoints[goalStateID]->coord;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

void GridEnvironment::reinit_smgs(int startStateID, int goalStateID, int start_ssp_idx, int goal_ssp_idx)
{
    for (GridState *s : m_states)
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
    GridState *start = new GridState;
    start->coord = m_ssp[start_ssp_idx]->coord;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    GridState *goal = new GridState;
    goal->coord = m_ssp[goal_ssp_idx]->coord;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

bool GridEnvironment::init(int **map, int row_size, int col_size, vector<pair<int, int>> actions, vector<double> costs, string planner)
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
    for (GridState *a : m_attractors)
    {

        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, GridState *> p : m_frontier_midpoints)
    {
        GridState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (GridState *s : m_ssp)
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
    stateMap = map;
    map_row_size = row_size;
    map_col_size = col_size;
    m_actions = actions;
    m_costs = costs;
    m_planner = planner;
    return true;
}

bool GridEnvironment::extractPath(
    string search,
    const std::vector<int> &idpath,
    std::vector<GridState *> &path)
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
