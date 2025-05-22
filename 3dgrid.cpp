#include <cmath>
#include <iostream>
#include <cstring>
#include <tuple>
#include "3dgrid.h"
#include "compares.h"
#include "ACLS.h"

auto std::hash<GridState>::operator()(
    const argument_type &s) const -> result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, (s.x));
    boost::hash_combine(seed, (s.y));
    boost::hash_combine(seed, (s.z));

    return seed;
}

Grid3DEnvironment::~Grid3DEnvironment()
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

GridState *Grid3DEnvironment::getHashEntry(int state_id) const
{
    return m_states[state_id];
}

void Grid3DEnvironment::printState(int state_id)
{
    GridState *state = getHashEntry(state_id);
    if (state != NULL)
    {
        printf("State %d: coord %d, %d, %d\n", state_id, state->x, state->y, state->z);
        return;
    }

    state = m_frontier_midpoints[state_id];
    if (state == NULL)
    {
        printf("State %d doesn't exist\n", state_id);
    }
    else
    {
        printf("midpoint: State %d: coord %d, %d, %d\n", state_id, state->x, state->y, state->z);
    }
}

void Grid3DEnvironment::printAttractors()
{
    printf("---------Printing attractors-----------\n");
    GridState *state;
    auto itr = m_attractors.begin();
    int i = 0;
    while (itr != m_attractors.end())
    {

        if ((*itr) != NULL)
        {
            printf("Key: %d Coord: %d, %d, %d\n", i, (*itr)->x, (*itr)->y, (*itr)->z);
        }
        itr++;
        i++;
    }
    printf("------------Printing complete-------------\n");
}

/// Return the state id of the state with the given coordinate or -1 if the
/// state has not yet been allocated.
int Grid3DEnvironment::getHashEntry(int x, int y, int z)
{
    GridState state;
    state.x = x;
    state.y = y;
    state.z = z;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end())
    {
        return -1;
    }
    return sit->second;
}

int Grid3DEnvironment::createHashEntry(
    int x, int y, int z)
{
    int state_id = reserveHashEntry();
    GridState *entry = getHashEntry(state_id);

    entry->x = x;
    entry->y = y;
    entry->z = z;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int Grid3DEnvironment::getOrCreateState(
    int x, int y, int z)
{
    int state_id = getHashEntry(x, y, z);
    if (state_id < 0)
    {
        state_id = createHashEntry(x, y, z);
    }
    return state_id;
}

int Grid3DEnvironment::reserveHashEntry()
{
    GridState *entry = new GridState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}

void Grid3DEnvironment::setGoalState(const Robot3DCoord &coord)
{
    if (m_goal_state_id == -1)
    {
        m_goal_state_id = createHashEntry(get<0>(coord), get<1>(coord), get<2>(coord));
    }
    m_states[m_goal_state_id]->x = get<0>(coord);
    m_states[m_goal_state_id]->y = get<1>(coord);
    m_states[m_goal_state_id]->z = get<2>(coord);
}

void Grid3DEnvironment::setStartState(const Robot3DCoord &coord)
{
    if (m_start_state_id == -1)
    {
        m_start_state_id = createHashEntry(get<0>(coord), get<1>(coord), get<2>(coord));
    }
    m_states[m_start_state_id]->x = get<0>(coord);
    m_states[m_start_state_id]->y = get<1>(coord);
    m_states[m_start_state_id]->z = get<2>(coord);
}

int Grid3DEnvironment::getStartStateID()
{
    return m_start_state_id;
}

int Grid3DEnvironment::getGoalStateID()
{
    return m_goal_state_id;
}

bool Grid3DEnvironment::coordFree(int x, int y, int z)
{
    int row = x;
    int col = y;
    int height = z;
    if (col >= 0 && col < map_col_size && row >= 0 && row < map_row_size && height >= 0 && height < map_height_size &&
        stateMap[row][col][height] == 0)
    {
        return true;
    }
    return false;
}

void Grid3DEnvironment::createAttractor(int state_id, int attr_id)
{
    GridState *attractor = new GridState;
    int x, y, z;

    if (m_planner == "ACLS" || m_planner == "FA*" || m_planner == "A*")
    {
        x = m_states[attr_id]->x;
        y = m_states[attr_id]->y;
        z = m_states[attr_id]->z;
    }
    else
    {
        x = m_states[state_id]->bp_x;
        y = m_states[state_id]->bp_y;
        z = m_states[state_id]->bp_z;
    }

    attractor->x = x;
    attractor->y = y;
    attractor->z = z;
    if (m_attractors.size() <= attr_id)
    {
        m_attractors.resize(attr_id + 1, nullptr);
    }
    m_attractors[attr_id] = attractor;
}

void Grid3DEnvironment::removeAttractor(int state_id)
{
    delete m_attractors[state_id];
    m_attractors[state_id] = nullptr;
}

double Grid3DEnvironment::GetAttractorDist(int state_id, int attr_id)
{

    return GetAttractorDist(m_states[state_id]->x, m_states[state_id]->y, m_states[state_id]->z, attr_id);
}

double Grid3DEnvironment::GetAttractorDist(int x, int y, int z, int attr_id)
{
    GridState *attr_state = m_attractors[attr_id];
    return m_weight * (abs(x - attr_state->x) + abs(y - attr_state->y) + abs(z - attr_state->z));

    // return sqrt(pow(x - attr_state->x, 2) + pow(y - attr_state->y, 2) + pow(z - attr_state->z, 2));
}

double Grid3DEnvironment::GetGoalHeuristic(int state_id)
{
    GridState *goal_state;
    GridState *state;

    goal_state = m_states[m_goal_state_id];
    state = m_states[state_id];

    return abs(goal_state->x - state->x) + abs(goal_state->y - state->y) + abs(goal_state->z - state->z);

    // return sqrt(pow(goal_state->x - state->x, 2) + pow(goal_state->y - state->y, 2) + pow(goal_state->z - state->z, 2));
}

void Grid3DEnvironment::GetSuccs(int state_id, bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    GridState *state = m_states[state_id];
    GridState *succ_state;
    int succ_state_id;
    int succ_x, succ_y, succ_z;

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
        succ_x = state->x + get<0>(m_actions[idx]);
        succ_y = state->y + get<1>(m_actions[idx]);
        succ_z = state->z + get<2>(m_actions[idx]);

        if (!coordFree(succ_x, succ_y, succ_z))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        succ_state_id = getOrCreateState(succ_x, succ_y, succ_z);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

void Grid3DEnvironment::GetSuccsAndBestPred(int state_id, bitset<NUM_OF_ACTIONS> action_operators, int attr_id, int bp_idx, bool *best_pred_match, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    GridState *state = m_states[state_id];
    GridState *succ_state;
    int succ_state_id;
    int succ_x, succ_y, succ_z;
    int best_x, best_y, best_z;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        succ_x = state->x + get<0>(m_actions[idx]);
        succ_y = state->y + get<1>(m_actions[idx]);
        succ_z = state->z + get<2>(m_actions[idx]);
        if (!coordFree(succ_x, succ_y, succ_z))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }

        // Calculating best pred
        // GridState *attr_state = m_attractors[attr_id];
        // curr_dist = m_weight * (abs(succ_x - attr_state->x) + abs(succ_y - attr_state->y) + abs(succ_z - attr_state->z));
        curr_dist = GetAttractorDist(succ_x, succ_y, succ_z, attr_id);

        if (curr_dist < best_pred_dist)
        {
            best_x = succ_x;
            best_y = succ_y;
            best_z = succ_z;
            best_pred_dist = curr_dist;
        }
        if (!action_operators.test(idx))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);

            continue;
        }

        succ_state_id = getOrCreateState(succ_x, succ_y, succ_z);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }

    if (best_x == m_states[state_id]->bp_x && best_y == m_states[state_id]->bp_y && best_z == m_states[state_id]->bp_z)
    {
        *best_pred_match = true;
    }
    else
    {
        *best_pred_match = false;
    }
}

int Grid3DEnvironment::getMatchingState(int state_id, int attr_id, int bp_id, int bp2_id, int bp2_attr_id)
{
    GridState *state = m_states[state_id];
    GridState *bp2_state = m_states[bp2_id];

    int x, y, z;
    x = state->bp_x;
    y = state->bp_y;
    z = state->bp_z;

    if (isGreaterThan(GetAttractorDist(x, y, z, attr_id), GetAttractorDist(bp2_id, bp2_attr_id)))
    {
        return bp_id;
    }
    else if (isLessThan(GetAttractorDist(x, y, z, attr_id), GetAttractorDist(bp2_id, bp2_attr_id)))
    {
        return bp2_id;
    }

    tuple<int, int, int> action1 = make_tuple(x - state->x, y - state->y, z - state->z);
    tuple<int, int, int> action2 = make_tuple(bp2_state->x - state->x, bp2_state->y - state->y, bp2_state->z - state->z);

    for (tuple<int, int, int> action : m_actions)
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

void Grid3DEnvironment::GetSuccs(int state_id, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    GridState *state = m_states[state_id];
    GridState *succ_state;
    int succ_state_id;
    int succ_x, succ_y, succ_z;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        succ_x = state->x + get<0>(m_actions[idx]);
        succ_y = state->y + get<1>(m_actions[idx]);
        succ_z = state->z + get<2>(m_actions[idx]);
        if (!coordFree(succ_x, succ_y, succ_z))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        succ_state_id = getOrCreateState(succ_x, succ_y, succ_z);
        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

int Grid3DEnvironment::GetBestPred(int state_id, int attr_id)
{
    GridState *state = getHashEntry(state_id);
    int best_pred_state_id;
    int succ_x, succ_y, succ_z;
    int best_x, best_y, best_z;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        succ_x = state->x + get<0>(m_actions[idx]);
        succ_y = state->y + get<1>(m_actions[idx]);
        succ_z = state->z + get<2>(m_actions[idx]);
        if (!coordFree(succ_x, succ_y, succ_z))
        {
            continue;
        }

        // GridState *attr_state = m_attractors[attr_id];
        // curr_dist = m_weight * (abs(succ_x - attr_state->x) + abs(succ_y - attr_state->y) + abs(succ_z - attr_state->z));
        curr_dist = GetAttractorDist(succ_x, succ_y, succ_z, attr_id);

        if (isLessThan(curr_dist, best_pred_dist))
        {
            best_x = succ_x;
            best_y = succ_y;
            best_z = succ_z;
            best_pred_dist = curr_dist;
        }
    }
    // Returns -1 if the pred state is not yet created
    best_pred_state_id = getHashEntry(best_x, best_y, best_z);
    return best_pred_state_id;
}

int Grid3DEnvironment::GetBestPredWithCreate(int state_id, int attr_id)
{
    GridState *state = getHashEntry(state_id);
    int best_pred_state_id;
    int succ_x, succ_y, succ_z;
    int best_x, best_y, best_z;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        succ_x = state->x + get<0>(m_actions[idx]);
        succ_y = state->y + get<1>(m_actions[idx]);
        succ_z = state->z + get<2>(m_actions[idx]);
        if (!coordFree(succ_x, succ_y, succ_z))
        {
            continue;
        }

        // GridState *attr_state = m_attractors[attr_id];
        // curr_dist = m_weight * (abs(succ_x - attr_state->x) + abs(succ_y - attr_state->y) + abs(succ_z - attr_state->z));
        curr_dist = GetAttractorDist(succ_x, succ_y, succ_z, attr_id);

        if (isLessThan(curr_dist, best_pred_dist))
        {
            best_x = succ_x;
            best_y = succ_y;
            best_z = succ_z;
            best_pred_dist = curr_dist;
        }
    }
    // Returns -1 if the pred state is not yet created
    best_pred_state_id = getHashEntry(best_x, best_y, best_z);
    if (best_pred_state_id < 0)
    {
        best_pred_state_id = createHashEntry(best_x, best_y, best_z);
    }
    return best_pred_state_id;
}

bool Grid3DEnvironment::sameState(int state_id, int attr_id)
{

    GridState *state = getHashEntry(state_id);
    GridState *attractor = m_attractors[attr_id];
    if (state == NULL || attractor == NULL)
    {
        return false;
    }
    return state->x == attractor->x && state->y == attractor->y && state->z == attractor->z;
}

void Grid3DEnvironment::DeleteState(int state_id)
{

    if (m_states[state_id] == NULL)
    {
        return;
    }
    m_state_to_id.erase(m_states[state_id]);
    delete m_states[state_id];
    m_states[state_id] = nullptr;
}

bool Grid3DEnvironment::isNeighbor(string search, int start_state_id, int goal_state_id)
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

    if (abs(start_state->x - goal_state->x) + abs(start_state->y - goal_state->y) + abs(start_state->z - goal_state->z) <= 1)
    {
        return true;
    }
    return false;
}

void Grid3DEnvironment::UpdateBP(int state_id, int bp_state_id)
{
    GridState *s = m_states[state_id];
    GridState *bp = m_states[bp_state_id];
    s->bp_x = bp->x;
    s->bp_y = bp->y;
    s->bp_z = bp->z;
}

void Grid3DEnvironment::AddMidpointState(int stateID, int newID)
{
    GridState *s = new GridState;
    if (m_temp_frontier_midpoints.empty())
    {
        s->x = m_states[stateID]->x;
        s->y = m_states[stateID]->y;
        s->z = m_states[stateID]->z;
    }
    else
    {
        s->x = m_temp_frontier_midpoints[stateID]->x;
        s->y = m_temp_frontier_midpoints[stateID]->y;
        s->z = m_temp_frontier_midpoints[stateID]->z;
    }

    m_frontier_midpoints[newID] = s;
}

void Grid3DEnvironment::AddTempMidpointState(int stateID)
{
    GridState *s = new GridState;
    s->x = m_states[stateID]->x;
    s->y = m_states[stateID]->y;
    s->z = m_states[stateID]->z;
    m_temp_frontier_midpoints[stateID] = s;
}

void Grid3DEnvironment::ClearTempMidpoints()
{
    for (auto p : m_temp_frontier_midpoints)
    {
        GridState *s = p.second;
        delete s;
    }
    m_temp_frontier_midpoints.clear();
}

void Grid3DEnvironment::reinit_frontier_search(int startStateID, int goalStateID)
{

    // To ensure consistency between planner and environment
    m_states.clear();
    m_state_to_id.clear();
    m_states.resize(max(startStateID, goalStateID) + 1, nullptr);
    GridState *start = new GridState;
    start->x = m_frontier_midpoints[startStateID]->x;
    start->y = m_frontier_midpoints[startStateID]->y;
    start->z = m_frontier_midpoints[startStateID]->z;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    GridState *goal = new GridState;
    goal->x = m_frontier_midpoints[goalStateID]->x;
    goal->y = m_frontier_midpoints[goalStateID]->y;
    goal->z = m_frontier_midpoints[goalStateID]->z;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

int Grid3DEnvironment::getInDegree(int state_id)
{
    GridState *state = m_states[state_id];
    int p = 0;
    int succ_x, succ_y, succ_z;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        succ_x = state->x + get<0>(m_actions[idx]);
        succ_y = state->y + get<1>(m_actions[idx]);
        succ_z = state->z + get<2>(m_actions[idx]);
        if (coordFree(succ_x, succ_y, succ_z))
        {
            p++;
        }
    }
    return p;
}

void Grid3DEnvironment::addSspState(int state_id)
{
    GridState *s = new GridState;
    s->x = m_states[state_id]->x;
    s->y = m_states[state_id]->y;
    s->z = m_states[state_id]->z;
    m_ssp.push_back(s);
}

void Grid3DEnvironment::reinit_smgs(int startStateID, int goalStateID, int start_ssp_idx, int goal_ssp_idx)
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
    start->x = m_ssp[start_ssp_idx]->x;
    start->y = m_ssp[start_ssp_idx]->y;
    start->z = m_ssp[start_ssp_idx]->z;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    GridState *goal = new GridState;
    goal->x = m_ssp[goal_ssp_idx]->x;
    goal->y = m_ssp[goal_ssp_idx]->y;
    goal->z = m_ssp[goal_ssp_idx]->z;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

bool Grid3DEnvironment::init(int ***map, int row_size, int col_size, int height_size, vector<tuple<int, int, int>> actions, vector<double> costs, string planner)
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
    map_height_size = height_size;
    m_actions = actions;
    m_costs = costs;
    m_planner = planner;
    return true;
}

bool Grid3DEnvironment::extractPath(
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
