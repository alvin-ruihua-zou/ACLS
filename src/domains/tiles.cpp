#include <cmath>
#include <iostream>
#include "tiles.h"
#include "ACLS.h"

auto std::hash<TileState>::operator()(
    const argument_type &s) const -> result_type
{
    std::size_t seed = 0;
    for (size_t i = 0; i < s.t2c_map.size(); i++)
    {
        boost::hash_combine(seed, s.t2c_map[i].first);
        boost::hash_combine(seed, s.t2c_map[i].second);
    }

    return seed;
}

TileMap TileEnvironment::t2c_2_tilemap(const TileToCoordMap &t2c_map)
{
    TileMap tile_map;
    for (size_t i = 0; i < t2c_map.size(); i++)
    {
        tile_map[t2c_map[i]] = i;
    }
    return tile_map;
}

TileEnvironment::~TileEnvironment()
{
    DeleteState(m_goal_state_id);
    for (auto *s : m_states)
    {
        if (s != NULL && s != nullptr)
        {
            delete s;
        }
    }

    for (TileState *a : m_attractors)
    {
        if (a != NULL)
        {
            delete a;
        }
    }

    for (pair<const int, TileState *> p : m_frontier_midpoints)
    {
        TileState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, TileState *> p : m_temp_frontier_midpoints)
    {
        TileState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }

    for (TileState *s : m_ssp)
    {
        delete s;
    }

    m_states.clear();
    m_state_to_id.clear();
    m_frontier_midpoints.clear();
    m_temp_frontier_midpoints.clear();
    m_ssp.clear();
}

TileState *TileEnvironment::getHashEntry(int state_id) const
{
    return m_states[state_id];
}

void TileEnvironment::printState(int state_id)
{
    TileState *state = getHashEntry(state_id);
    TileMap tile_map = t2c_2_tilemap(state->t2c_map);
    if (state != NULL)
    {
        printf("------------------\n");
        for (int row = 0; row < map_row_size; row++)
        {
            for (int col = 0; col < map_col_size; col++)
            {
                printf("%d ", tile_map[make_pair(row, col)]);
            }
            printf("\n");
        }
        printf("------------------\n");
        return;
    }
    state = m_frontier_midpoints[state_id];
    if (state == NULL)
    {
        printf("State %d doesn't exist\n", state_id);
    }
    else
    {
        printf("------------------\n");
        for (int row = 0; row < map_row_size; row++)
        {
            for (int col = 0; col < map_col_size; col++)
            {
                printf("%d ", tile_map[make_pair(row, col)]);
            }
            printf("\n");
        }
        printf("------------------\n");
    }
}

void TileEnvironment::printAttractors()
{
    printf("---------Printing attractors-----------\n");
    TileState *state;
    auto itr = m_attractors.begin();
    int i = 0;
    while (itr != m_attractors.end())
    {
        state = *itr;
        if (state != NULL)
        {
            TileMap tile_map = t2c_2_tilemap(state->t2c_map);
            printf("Key: %d, tile_map:\n", i);
            printf("------------------\n");
            for (int row = 0; row < map_row_size; row++)
            {
                for (int col = 0; col < map_col_size; col++)
                {
                    printf("%d ", tile_map[make_pair(row, col)]);
                }
                printf("\n");
            }
            printf("------------------\n");
        }
        itr++;
        i++;
    }
    printf("------------Printing complete-------------\n");
}

TileToCoordMap TileEnvironment::tile_2_t2c(const TileMap &tile_map)
{
    TileToCoordMap t2c_map(tile_map.size(), make_pair(0, 0));
    for (auto c : tile_map)
    {
        t2c_map[c.second] = c.first;
    }
    return t2c_map;
}

/// Return the state id of the state or -1 if the
/// state has not yet been allocated.
int TileEnvironment::getHashEntry(const TileToCoordMap &t2c_map)
{
    TileState state;
    state.t2c_map = t2c_map;
    auto sit = m_state_to_id.find(&state);
    if (sit == m_state_to_id.end())
    {
        return -1;
    }
    return sit->second;
}

int TileEnvironment::createHashEntry(
    const TileToCoordMap &t2c_map)
{
    int state_id = reserveHashEntry();
    TileState *entry = getHashEntry(state_id);

    entry->t2c_map = t2c_map;
    entry->bp_t2c_map = t2c_map;

    // map state -> state id
    m_state_to_id[entry] = state_id;

    return state_id;
}

int TileEnvironment::getOrCreateState(
    const TileToCoordMap &t2c_map)
{
    int state_id = getHashEntry(t2c_map);
    if (state_id < 0)
    {
        state_id = createHashEntry(t2c_map);
    }

    return state_id;
}

int TileEnvironment::reserveHashEntry()
{
    TileState *entry = new TileState;
    int state_id = (int)m_states.size();

    // map state id -> state
    m_states.push_back(entry);

    return state_id;
}

void TileEnvironment::setGoalState(const TileMap &tile_map)
{

    TileToCoordMap t2c_map(tile_map.size(), make_pair(0, 0));
    for (pair<TileCoord, int> c : tile_map)
    {
        t2c_map[c.second] = c.first;
    }
    if (m_goal_state_id == -1)
    {
        m_goal_state_id = createHashEntry(t2c_map);
    }
    m_states[m_goal_state_id]->t2c_map = t2c_map;

    m_state_to_id[m_states[m_goal_state_id]] = m_goal_state_id;
}

void TileEnvironment::setStartState(const TileMap &tile_map)
{

    TileToCoordMap t2c_map(tile_map.size(), make_pair(0, 0));
    for (pair<TileCoord, int> c : tile_map)
    {
        t2c_map[c.second] = c.first;
    }
    if (m_start_state_id == -1)
    {
        m_start_state_id = createHashEntry(t2c_map);
    }
    m_states[m_start_state_id]->t2c_map = t2c_map;
    m_state_to_id[m_states[m_start_state_id]] = m_start_state_id;
}

int TileEnvironment::getStartStateID()
{
    return m_start_state_id;
}

int TileEnvironment::getGoalStateID()
{
    return m_goal_state_id;
}

bool TileEnvironment::validCoord(const TileCoord &coord)
{
    int row = coord.first;
    int col = coord.second;
    if (col >= 0 && col < map_col_size && row >= 0 && row < map_row_size)
    {
        return true;
    }
    return false;
}

void TileEnvironment::createAttractor(int state_id, int attr_id)
{
    TileState *attractor = new TileState;
    if (m_planner == "ACLS" || m_planner == "FA*" || m_planner == "A*")
    {
        attractor->t2c_map = m_states[attr_id]->t2c_map;
    }
    else
    {
        attractor->t2c_map = m_states[state_id]->bp_t2c_map;
    }
    if (m_attractors.size() <= attr_id)
    {
        m_attractors.resize(attr_id + 1, nullptr);
    }
    m_attractors[attr_id] = attractor;
}

void TileEnvironment::removeAttractor(int state_id)
{
    delete m_attractors[state_id];
    m_attractors[state_id] = nullptr;
}

double TileEnvironment::GetAttractorDist(int state_id, int attr_id)
{
    TileToCoordMap t2c_map = m_states[state_id]->t2c_map;
    return GetAttractorDist(t2c_map, attr_id);
}

double TileEnvironment::GetAttractorDist(const TileToCoordMap &t2c_map, int attr_id)
{
    TileToCoordMap attr_t2c_map = m_attractors[attr_id]->t2c_map;
    double dist = 0;

    for (size_t idx = 0; idx < t2c_map.size(); idx++)
    {
        if (t2c_map[idx] != attr_t2c_map[idx])
        {
            dist += 1;
        }
    }
    return dist;
}

// double TileEnvironment::GetAttractorDist(const TileToCoordMap &t2c_map, int attr_id)
// {
//     TileToCoordMap attr_t2c_map = m_attractors[attr_id]->t2c_map;
//     double manhattan_dist = 0;
//     int linear_conflicts = 0;

//     // Calculate Manhattan Distance
//     for (size_t idx = 0; idx < t2c_map.size(); idx++)
//     {
//         manhattan_dist += abs(t2c_map[idx].first - attr_t2c_map[idx].first) +
//                           abs(t2c_map[idx].second - attr_t2c_map[idx].second);
//     }

//     // Calculate Linear Conflicts
//     // Check rows for conflicts
//     for (int row = 0; row < map_row_size; row++)
//     {
//         std::vector<int> tiles_in_row;
//         for (int col = 0; col < map_col_size; col++)
//         {
//             for (size_t idx = 0; idx < t2c_map.size(); idx++)
//             {
//                 if (t2c_map[idx] == std::make_pair(row, col))
//                 {
//                     tiles_in_row.push_back(idx);
//                     break;
//                 }
//             }
//         }

//         for (size_t i = 0; i < tiles_in_row.size(); i++)
//         {
//             for (size_t j = i + 1; j < tiles_in_row.size(); j++)
//             {
//                 int tile1 = tiles_in_row[i];
//                 int tile2 = tiles_in_row[j];

//                 // Check if both tiles are in their goal row and conflict
//                 if (attr_t2c_map[tile1].first == row &&
//                     attr_t2c_map[tile2].first == row &&
//                     attr_t2c_map[tile1].second > attr_t2c_map[tile2].second)
//                 {
//                     linear_conflicts++;
//                 }
//             }
//         }
//     }

//     // Check columns for conflicts
//     for (int col = 0; col < map_col_size; col++)
//     {
//         std::vector<int> tiles_in_col;
//         for (int row = 0; row < map_row_size; row++)
//         {
//             for (size_t idx = 0; idx < t2c_map.size(); idx++)
//             {
//                 if (t2c_map[idx] == std::make_pair(row, col))
//                 {
//                     tiles_in_col.push_back(idx);
//                     break;
//                 }
//             }
//         }

//         for (size_t i = 0; i < tiles_in_col.size(); i++)
//         {
//             for (size_t j = i + 1; j < tiles_in_col.size(); j++)
//             {
//                 int tile1 = tiles_in_col[i];
//                 int tile2 = tiles_in_col[j];

//                 // Check if both tiles are in their goal column and conflict
//                 if (attr_t2c_map[tile1].second == col &&
//                     attr_t2c_map[tile2].second == col &&
//                     attr_t2c_map[tile1].first > attr_t2c_map[tile2].first)
//                 {
//                     linear_conflicts++;
//                 }
//             }
//         }
//     }

//     // Combine Manhattan Distance and Linear Conflicts
//     return manhattan_dist + 2 * linear_conflicts;
// }

double TileEnvironment::GetGoalHeuristic(int state_id)
{
    double manhattan_dist = 0;
    int linear_conflicts = 0;

    // Calculate Manhattan Distance
    for (size_t idx = 0; idx < m_states[state_id]->t2c_map.size(); idx++)
    {
        manhattan_dist += abs(m_states[state_id]->t2c_map[idx].first - m_states[m_goal_state_id]->t2c_map[idx].first) +
                          abs(m_states[state_id]->t2c_map[idx].second - m_states[m_goal_state_id]->t2c_map[idx].second);
    }

    // Calculate Linear Conflicts
    // Check rows for conflicts
    for (int row = 0; row < map_row_size; row++)
    {
        std::vector<int> tiles_in_row;
        for (int col = 0; col < map_col_size; col++)
        {
            for (size_t idx = 0; idx < m_states[state_id]->t2c_map.size(); idx++)
            {
                if (m_states[state_id]->t2c_map[idx] == std::make_pair(row, col))
                {
                    tiles_in_row.push_back(idx);
                    break;
                }
            }
        }

        for (size_t i = 0; i < tiles_in_row.size(); i++)
        {
            for (size_t j = i + 1; j < tiles_in_row.size(); j++)
            {
                int tile1 = tiles_in_row[i];
                int tile2 = tiles_in_row[j];

                // Check if both tiles are in their goal row and conflict
                if (m_states[m_goal_state_id]->t2c_map[tile1].first == row &&
                    m_states[m_goal_state_id]->t2c_map[tile2].first == row &&
                    m_states[m_goal_state_id]->t2c_map[tile1].second > m_states[m_goal_state_id]->t2c_map[tile2].second)
                {
                    linear_conflicts++;
                }
            }
        }
    }

    // Check columns for conflicts
    for (int col = 0; col < map_col_size; col++)
    {
        std::vector<int> tiles_in_col;
        for (int row = 0; row < map_row_size; row++)
        {
            for (size_t idx = 0; idx < m_states[state_id]->t2c_map.size(); idx++)
            {
                if (m_states[state_id]->t2c_map[idx] == std::make_pair(row, col))
                {
                    tiles_in_col.push_back(idx);
                    break;
                }
            }
        }

        for (size_t i = 0; i < tiles_in_col.size(); i++)
        {
            for (size_t j = i + 1; j < tiles_in_col.size(); j++)
            {
                int tile1 = tiles_in_col[i];
                int tile2 = tiles_in_col[j];

                // Check if both tiles are in their goal column and conflict
                if (m_states[m_goal_state_id]->t2c_map[tile1].second == col &&
                    m_states[m_goal_state_id]->t2c_map[tile2].second == col &&
                    m_states[m_goal_state_id]->t2c_map[tile1].first > m_states[m_goal_state_id]->t2c_map[tile2].first)
                {
                    linear_conflicts++;
                }
            }
        }
    }

    // Combine Manhattan Distance and Linear Conflicts
    return manhattan_dist + 2 * linear_conflicts;
}

void TileEnvironment::GetSuccs(int state_id, bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    TileState *state = m_states[state_id];
    TileToCoordMap succ_state_map;
    int succ_state_id;
    TileCoord new_empty_coord, old_empty_coord;
    int moved_tile;

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
        succ_state_map = state->t2c_map;
        // Succ state swaps position of empty tile and moved tile.
        new_empty_coord = state->t2c_map[0] + m_actions[idx];
        if (!validCoord(new_empty_coord))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        old_empty_coord = succ_state_map[0];
        for (size_t i = 0; i < succ_state_map.size(); i++)
        {
            if (succ_state_map[i] == new_empty_coord)
            {
                succ_state_map[i] = old_empty_coord;
                break;
            }
        }
        succ_state_map[0] = new_empty_coord;

        succ_state_id = getOrCreateState(succ_state_map);

        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

void TileEnvironment::GetSuccs(int state_id, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    TileState *state = m_states[state_id];
    TileToCoordMap succ_state_map;
    int succ_state_id;
    TileCoord new_empty_coord, old_empty_coord;
    int moved_tile;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        succ_state_map = state->t2c_map;
        // Succ state swaps position of empty tile and moved tile.
        new_empty_coord = state->t2c_map[0] + m_actions[idx];
        if (!validCoord(new_empty_coord))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        old_empty_coord = succ_state_map[0];
        for (size_t i = 0; i < succ_state_map.size(); i++)
        {
            if (succ_state_map[i] == new_empty_coord)
            {
                succ_state_map[i] = old_empty_coord;
                break;
            }
        }
        succ_state_map[0] = new_empty_coord;

        succ_state_id = getOrCreateState(succ_state_map);

        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
}

int TileEnvironment::GetBestPred(int state_id, int attr_id)
{
    TileState *state = m_states[state_id];
    int best_pred_state_id;
    TileToCoordMap pred_t2c_map;
    TileToCoordMap best_pred_t2c_map = state->t2c_map;
    TileCoord old_empty_coord, new_empty_coord;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        pred_t2c_map = state->t2c_map;
        old_empty_coord = state->t2c_map[0];
        new_empty_coord = old_empty_coord + m_actions[idx];

        if (!validCoord(new_empty_coord))
        {
            continue;
        }
        // swap tiles in t2c_map
        for (size_t i = 0; i < pred_t2c_map.size(); i++)
        {
            if (pred_t2c_map[i] == new_empty_coord)
            {
                pred_t2c_map[i] = old_empty_coord;
                break;
            }
        }
        pred_t2c_map[0] = new_empty_coord;
        curr_dist = GetAttractorDist(pred_t2c_map, attr_id);

        if (curr_dist < best_pred_dist)
        {
            best_pred_t2c_map = pred_t2c_map;
            best_pred_dist = curr_dist;
        }
    }

    best_pred_state_id = getHashEntry(best_pred_t2c_map);

    return best_pred_state_id;
}

int TileEnvironment::GetBestPredWithCreate(int state_id, int attr_id)
{

    TileState *state = getHashEntry(state_id);
    int best_pred_state_id;
    TileToCoordMap pred_t2c_map;
    TileToCoordMap best_pred_t2c_map = state->t2c_map;
    TileCoord old_empty_coord, new_empty_coord;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {
        pred_t2c_map = state->t2c_map;
        old_empty_coord = state->t2c_map[0];
        new_empty_coord = old_empty_coord + m_actions[idx];

        if (!validCoord(new_empty_coord))
        {
            continue;
        }
        // swap tiles in t2c_map
        for (size_t i = 0; i < pred_t2c_map.size(); i++)
        {
            if (pred_t2c_map[i] == new_empty_coord)
            {
                pred_t2c_map[i] = old_empty_coord;
                break;
            }
        }
        pred_t2c_map[0] = new_empty_coord;
        curr_dist = GetAttractorDist(pred_t2c_map, attr_id);

        if (curr_dist < best_pred_dist)
        {
            best_pred_t2c_map = pred_t2c_map;
            best_pred_dist = curr_dist;
        }
    }
    // Returns -1 if the pred state is not yet created
    best_pred_state_id = getHashEntry(best_pred_t2c_map);

    if (best_pred_state_id < 0)
    {
        best_pred_state_id = createHashEntry(best_pred_t2c_map);
    }
    return best_pred_state_id;
}

void TileEnvironment::GetSuccsAndBestPred(int state_id, bitset<NUM_OF_ACTIONS> action_operators, int attr_id, int bp_idx, bool *best_pred_match, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions)
{
    TileState *state = m_states[state_id];
    int succ_state_id;
    TileToCoordMap pred_t2c_map;
    TileToCoordMap best_pred_t2c_map = state->t2c_map;
    TileCoord old_empty_coord, new_empty_coord;
    double best_pred_dist = INFINITECOST;
    double curr_dist;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        pred_t2c_map = state->t2c_map;
        old_empty_coord = state->t2c_map[0];
        new_empty_coord = old_empty_coord + m_actions[idx];

        if (!validCoord(new_empty_coord))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }
        // swap tiles in t2c_map
        for (size_t i = 0; i < pred_t2c_map.size(); i++)
        {
            if (pred_t2c_map[i] == new_empty_coord)
            {
                pred_t2c_map[i] = old_empty_coord;
                break;
            }
        }
        pred_t2c_map[0] = new_empty_coord;
        curr_dist = GetAttractorDist(pred_t2c_map, attr_id);

        if (curr_dist < best_pred_dist)
        {
            best_pred_t2c_map = pred_t2c_map;
            best_pred_dist = curr_dist;
        }

        if (!action_operators.test(idx))
        {
            succs->push_back(-1);
            costs->push_back(-1);
            actions->push_back(-1);
            continue;
        }

        succ_state_id = getOrCreateState(pred_t2c_map);

        succs->push_back(succ_state_id);
        costs->push_back(m_costs[idx]);
        actions->push_back(idx);
    }
    if (best_pred_t2c_map == m_states[state_id]->bp_t2c_map)
    {
        *best_pred_match = true;
    }
    else
    {
        *best_pred_match = false;
    }
}

int TileEnvironment::getMatchingState(int state_id, int attr_id, int bp_id, int bp2_id, int bp2_attr_id)
{
    TileState *state = m_states[state_id];
    TileState *bp2_state = m_states[bp2_id];

    if (GetAttractorDist(state->bp_t2c_map, attr_id) < GetAttractorDist(bp2_id, bp2_attr_id))
    {
        return bp2_id;
    }
    else if (GetAttractorDist(state->bp_t2c_map, attr_id) > GetAttractorDist(bp2_id, bp2_attr_id))
    {
        return bp_id;
    }

    pair<int, int> action1 = state->bp_t2c_map[0] - state->t2c_map[0];
    pair<int, int> action2 = bp2_state->t2c_map[0] - state->t2c_map[0];
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

int TileEnvironment::getInDegree(int state_id)
{
    TileState *state = m_states[state_id];
    int p = 0;
    TileCoord old_empty_coord, new_empty_coord;

    // idx is the index of the action
    for (size_t idx = 0; idx < m_actions.size(); ++idx)
    {

        TileToCoordMap pred_t2c_map = state->t2c_map;
        old_empty_coord = state->t2c_map[0];
        new_empty_coord = old_empty_coord + m_actions[idx];

        if (validCoord(new_empty_coord))
        {
            p++;
        }
    }
    return p;
}

void TileEnvironment::addSspState(int state_id)
{
    TileState *s = new TileState;
    s->t2c_map = m_states[state_id]->t2c_map;
    m_ssp.push_back(s);
}

bool TileEnvironment::sameState(int state_id, int attr_id)
{

    TileState *state = getHashEntry(state_id);
    TileState *attractor = m_attractors[attr_id];
    if (state == NULL || attractor == NULL)
    {
        return false;
    }
    return state->t2c_map == attractor->t2c_map;
}

void TileEnvironment::DeleteState(int state_id)
{

    if (m_states[state_id] == NULL)
    {
        return;
    }
    m_state_to_id.erase(m_states[state_id]);
    delete m_states[state_id];
    m_states[state_id] = nullptr;
}

bool TileEnvironment::isNeighbor(string search, int start_state_id, int goal_state_id)
{
    TileState *start_state, *goal_state;
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
    int diff = 0;
    TileCoord coord_diff;
    for (size_t i = 0; i < start_state->t2c_map.size(); i++)
    {
        coord_diff = start_state->t2c_map[i] - goal_state->t2c_map[i]; // Diff of coordinate for empty cell
        diff += abs(coord_diff.first) + abs(coord_diff.second);
    }

    if (diff <= 2)
    {
        return true;
    }
    return false;
}

void TileEnvironment::UpdateBP(int state_id, int bp_state_id)
{
    TileState *s = m_states[state_id];
    TileState *bp = m_states[bp_state_id];
    s->bp_t2c_map = bp->t2c_map;
}

void TileEnvironment::AddMidpointState(int stateID, int newID)
{
    TileState *s = new TileState;
    if (m_temp_frontier_midpoints.empty())
    {
        s->t2c_map = m_states[stateID]->t2c_map;
    }
    else
    {
        s->t2c_map = m_temp_frontier_midpoints[stateID]->t2c_map;
    }

    m_frontier_midpoints[newID] = s;
    return;
}

void TileEnvironment::AddTempMidpointState(int stateID)
{
    TileState *s = new TileState;
    s->t2c_map = m_states[stateID]->t2c_map;
    m_temp_frontier_midpoints[stateID] = s;
}

void TileEnvironment::ClearTempMidpoints()
{
    for (auto p : m_temp_frontier_midpoints)
    {
        TileState *s = p.second;
        delete s;
    }
    m_temp_frontier_midpoints.clear();
}

void TileEnvironment::reinit_frontier_search(int startStateID, int goalStateID)
{

    // To ensure consistency between planner and environment
    m_states.clear();
    m_state_to_id.clear();
    m_states.resize(max(startStateID, goalStateID) + 1, nullptr);
    TileState *start = new TileState;
    start->t2c_map = m_frontier_midpoints[startStateID]->t2c_map;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    TileState *goal = new TileState;
    goal->t2c_map = m_frontier_midpoints[goalStateID]->t2c_map;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

void TileEnvironment::reinit_smgs(int startStateID, int goalStateID, int start_ssp_idx, int goal_ssp_idx)
{
    for (TileState *s : m_states)
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
    TileState *start = new TileState;
    start->t2c_map = m_ssp[start_ssp_idx]->t2c_map;
    m_states[startStateID] = start;
    m_state_to_id[start] = startStateID;
    m_start_state_id = startStateID;
    TileState *goal = new TileState;
    goal->t2c_map = m_ssp[goal_ssp_idx]->t2c_map;
    m_states[goalStateID] = goal;
    m_state_to_id[goal] = goalStateID;
    m_goal_state_id = goalStateID;
}

bool TileEnvironment::init(int row_size, int col_size, vector<pair<int, int>> actions, vector<double> costs, string planner)
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

    for (TileState *a : m_attractors)
    {

        if (a != NULL)
        {
            delete a;
        }
    }
    for (pair<const int, TileState *> p : m_frontier_midpoints)
    {
        TileState *a = p.second;
        if (a != NULL)
        {
            delete a;
        }
    }
    for (TileState *s : m_ssp)
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
    map_row_size = row_size;
    map_col_size = col_size;
    m_actions = actions;
    m_costs = costs;
    m_planner = planner;
    return true;
}

bool TileEnvironment::extractPath(
    string search,
    const std::vector<int> &idpath,
    std::vector<TileState *> &path)
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
