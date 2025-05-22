#ifndef GRID_2D_ENVIRONMENT_H_
#define GRID_2D_ENVIRONMENT_H_

#include <unordered_map>
#include <vector>
#include <utility>
#include <bitset>
#include <cstring>
#include "environment.h"

#include <boost/functional/hash.hpp>
using namespace std;

typedef pair<int, int> RobotCoord;

/**
 * @struct GridState
 * @brief Represents a state in the 2D grid environment.
 */
struct GridState
{
    RobotCoord coord;    ///< Coordinates of the state.
    RobotCoord bp_coord; ///< Coordinates of parent of the state.
};
/**
 * @brief Overloaded + operators for RobotCoord.
 */
template <typename T, typename U>
std::pair<T, U> operator+(const std::pair<T, U> &l, const std::pair<T, U> &r)
{
    return {l.first + r.first, l.second + r.second};
};

/**
 * @brief Overloaded - operators for RobotCoord.
 */
template <typename T, typename U>
std::pair<T, U> operator-(const std::pair<T, U> &l, const std::pair<T, U> &r)
{
    return {l.first - r.first, l.second - r.second};
};

/**
 * @brief Equality operator for GridState.
 * @param a The first GridState.
 * @param b The second GridState.
 * @return True if the two states are equal, false otherwise.
 */
inline bool operator==(const GridState &a, const GridState &b)
{
    return a.coord == b.coord;
}

/**
 * @brief Hash function specialization for GridState.
 */
namespace std
{
    template <>
    struct hash<GridState>
    {
        typedef GridState argument_type;
        typedef std::size_t result_type;
        result_type operator()(const argument_type &s) const;
    };
}

/**
 * @struct StateHash
 * @brief Hash function for GridState pointers.
 */
struct StateHash
{
    /**
     * @brief Hash function for GridState pointers.
     * @param state Pointer to the GridState.
     * @return The hash value.
     */
    std::size_t operator()(const GridState *state) const
    {
        // Hash only the coord field
        std::size_t seed = 0;
        boost::hash_combine(seed, state->coord.first);
        boost::hash_combine(seed, state->coord.second);
        return seed;
    }
};

/**
 * @struct StateEqual
 * @brief Equality comparator for GridState pointers.
 */
struct StateEqual
{
    /**
     * @brief Equality comparator for GridState pointers.
     * @param lhs Pointer to the first GridState.
     * @param rhs Pointer to the second GridState.
     * @return True if the two states are equal, false otherwise.
     */
    bool operator()(const GridState *lhs, const GridState *rhs) const
    {
        // Compare only the coord field
        return lhs->coord == rhs->coord;
    }
};

/**
 * @class GridEnvironment
 * @brief Represents a 2D grid environment for planning.
 */
class GridEnvironment : public DiscreteSpaceInformation
{
public:
    int **stateMap;   ///< 2D array representing the grid map.
    int map_row_size; ///< Number of rows in the grid.
    int map_col_size; ///< Number of columns in the grid.

    int m_goal_state_id = -1;  ///< ID of the goal state.
    int m_start_state_id = -1; ///< ID of the start state.

    int m_weight; ///< Weight for heuristic calculations.

    template <
        class Key,
        class T,
        class Hash = std::hash<Key>,
        class KeyEqual = std::equal_to<Key>,
        class Allocator = std::allocator<std::pair<const Key, T>>>
    using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

    typedef GridState StateKey;
    hash_map<StateKey *, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords
    std::vector<GridState *> m_states;     ///< Maps state IDs to GridState pointers.
    std::vector<GridState *> m_attractors; ///< List of attractor states.

    // Frontier Search
    typedef unordered_map<int, GridState *> frontier_midpoints_t;
    frontier_midpoints_t m_frontier_midpoints;      ///< Maps state IDs to frontier midpoint states.
    frontier_midpoints_t m_temp_frontier_midpoints; ///< Maps state IDs to temporary frontier midpoint states.

    // SMGS
    vector<GridState *> m_ssp; ///< Sparse solution path states.

    std::vector<std::pair<int, int>> m_actions; ///< List of possible actions.
    std::vector<double> m_costs;                ///< List of costs for each action.
    string m_planner;                           ///< Name of the planner being used.

    /**
     * @brief Destructor for the GridEnvironment class.
     */
    ~GridEnvironment();

    /**
     * @brief Prints information about a specific state.
     * @param state_id The ID of the state to print.
     */
    void printState(int state_id) override;

    /**
     * @brief Prints information about all attractors.
     */
    void printAttractors() override;

    /**
     * @brief Retrieves the GridState corresponding to a state ID.
     * @param state_id The ID of the state.
     * @return Pointer to the corresponding GridState.
     */
    GridState *getHashEntry(int state_id) const;

    /**
     * @brief Retrieves the state ID for a given coordinate.
     * @param coord The coordinate of the state.
     * @return The state ID, or -1 if the state does not exist.
     */
    int getHashEntry(const RobotCoord &coord);

    /**
     * @brief Creates a new state for a given coordinate.
     * @param coord The coordinate of the state.
     * @return The state ID of the newly created state.
     */
    int createHashEntry(const RobotCoord &coord);

    /**
     * @brief Retrieves or creates a state for a given coordinate.
     * @param coord The coordinate of the state.
     * @return The state ID of the retrieved or newly created state.
     */
    int getOrCreateState(const RobotCoord &coord);

    /**
     * @brief Reserves a new state ID in m_states.
     * @return The reserved state ID.
     */
    int reserveHashEntry();

    /**
     * @brief Sets the goal state.
     * @param coord The coordinate of the goal state.
     */
    void setGoalState(const RobotCoord &coord);

    /**
     * @brief Sets the start state.
     * @param coord The coordinate of the start state.
     */
    void setStartState(const RobotCoord &coord);

    /**
     * @brief Retrieves the state ID of the start state.
     * @return The state ID of the start state.
     */
    int getStartStateID();

    /**
     * @brief Retrieves the state ID of the goal state.
     * @return The state ID of the goal state.
     */
    int getGoalStateID();

    /**
     * @brief Checks if a coordinate is free (not an obstacle).
     * @param coord The coordinate to check.
     * @return True if the coordinate is free, false otherwise.
     */
    bool coordFree(const RobotCoord &coord);

    /**
     * @brief Get the distance between the state and the attractor.
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     */
    double GetAttractorDist(int state_id, int attr_id) override;

    /**
     * @brief Get the distance between the state and the attractor.
     * @param coord The coordinates of the state.
     * @param attr_id The ID of the attractor.
     */
    double GetAttractorDist(const RobotCoord &coord, int attr_id);

    /**
     * @brief Creates an attractor for a given state.
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     */
    void createAttractor(int state_id, int attr_id) override;

    /**
     * @brief Removes an attractor.
     * @param state_id The ID of the attractor to remove.
     */
    void removeAttractor(int state_id) override;

    /**
     * @brief Checks if two states are the same.
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @return True if the states are the same, false otherwise.
     */
    bool sameState(int state_id, int attr_id) override;

    /**
     * @brief Checks if the start and goal states are neighbors.
     * @param search The search type (e.g., "frontier", "smgs").
     * @param start_state_id The ID of the start state.
     * @param goal_state_id The ID of the goal state.
     * @return True if the states are neighbors, false otherwise.
     */
    bool isNeighbor(string search, int start_state_id, int goal_state_id) override;

    /**
     * @brief Retrieves the heuristic estimate to the goal state.
     * @param state_id The ID of the state.
     * @return The heuristic estimate.
     */
    double GetGoalHeuristic(int state_id) override;

    /**
     * @brief Retrieves the successors of a state with action operators.
     * @param state_id The ID of the state.
     * @param action_operators A bitset representing the available actions.
     * @param succs Vector to store successor state IDs.
     * @param costs Vector to store the costs of transitions to successors.
     * @param actions Vector to store the action IDs leading to successors.
     */
    void GetSuccs(int state_id, bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions) override;

    /**
     * @brief Retrieves the successors of a state.
     * @param state_id The ID of the state.
     * @param succs Vector to store successor state IDs.
     * @param costs Vector to store the costs of transitions to successors.
     * @param actions Vector to store the action IDs leading to successors.
     */
    void GetSuccs(int state_id, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions) override;

    /**
     * @brief Retrieves the successors of a state and determines the best predecessor.
     *
     * @param state_id The ID of the state.
     * @param action_operators A bitset representing the available actions.
     * @param attr_id The ID of the attractor.
     * @param bp_idx The index of the parent.
     * @param best_pred_match Pointer to a boolean indicating if the best predecessor matches.
     * @param succs Vector to store successor state IDs.
     * @param costs Vector to store the costs of transitions to successors.
     * @param actions Vector to store the action IDs leading to successors.
     */
    void GetSuccsAndBestPred(int state_id, bitset<NUM_OF_ACTIONS> action_operators, int attr_id, int bp_idx, bool *best_pred_match, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions) override;

    /**
     * @brief Retrieves the best predecessor for a given state and attractor.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @return The ID of the best predecessor state.
     */
    int GetBestPred(int state_id, int attr_id) override;

    /**
     * @brief Retrieves the best predecessor for a given state and attractor, creating it if necessary.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @return The ID of the best predecessor state.
     */
    int GetBestPredWithCreate(int state_id, int attr_id) override;

    /**
     * @brief Deletes a state from the environment.
     *
     * @param state_id The ID of the state to delete.
     */
    void DeleteState(int state_id) override;

    /**
     * @brief Returns the matching state based on tie breaking.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @param bp_id The ID of the parent of the state specified by state_id.
     * @param bp2_id The ID of the parent of the other state.
     * @param bp2_attr_id The ID of the second attractor.
     * @return The ID of the matching state.
     */
    int getMatchingState(int state_id, int attr_id, int bp_id, int bp2_id, int bp2_attr_id) override;

    /**
     * @brief Updates the parent of a state.
     *
     * @param state_id The ID of the state.
     * @param bp_state_id The ID of the parent of the state specified by state_id.
     */
    void UpdateBP(int state_id, int bp_state_id) override;

    /////////////////////////////////////////////////////////////////////////
    // Frontier Search
    /////////////////////////////////////////////////////////////////////////

    /**
     * @brief Adds a new midpoint state.
     *
     * @param stateID The ID of the state.
     * @param newID The ID of the new midpoint state.
     */
    void AddMidpointState(int stateID, int newID) override;

    /**
     * @brief Adds a temporary midpoint state.
     *
     * @param stateID The ID of the state.
     */
    void AddTempMidpointState(int stateID) override;

    /**
     * @brief Clears all temporary midpoint states.
     */
    void ClearTempMidpoints() override;

    /**
     * @brief Reinitializes the frontier search with a new start and goal state.
     *
     * @param startStateID The ID of the start state.
     * @param goalStateID The ID of the goal state.
     */
    void reinit_frontier_search(int startStateID, int goalStateID) override;

    /////////////////////////////////////////////////////////////////////////
    // SMGS
    /////////////////////////////////////////////////////////////////////////

    /**
     * @brief Retrieves the in-degree of a state.
     *
     * @param state_id The ID of the state.
     * @return The in-degree of the state.
     */
    int getInDegree(int state_id) override;

    /**
     * @brief Adds a state to the sparse solution path (SSP).
     *
     * @param state_id The ID of the state.
     */
    void addSspState(int state_id) override;

    /**
     * @brief Reinitializes the Sparse Multi-Goal Search (SMGS) with new start and goal states.
     *
     * @param startStateID The ID of the start state.
     * @param goalStateID The ID of the goal state.
     * @param start_ssp_idx The index of the start state in the SSP.
     * @param goal_ssp_idx The index of the goal state in the SSP.
     */
    void reinit_smgs(int startStateID, int goalStateID, int start_ssp_idx, int goal_ssp_idx) override;

    /**
     * @brief Retrieves the total number of states in the environment.
     *
     * @return The total number of states.
     */
    inline int getStateNum() override
    {
        return m_states.size();
    }

    /**
     * @brief Sets the weight for heuristic calculations.
     *
     * @param w The weight value.
     */
    inline void setWeight(int w)
    {
        m_weight = w;
    }

    /**
     * @brief Initializes the environment with a grid map.
     * @param map The 2D grid map.
     * @param row_size The number of rows in the map.
     * @param col_size The number of columns in the map.
     * @param actions The list of possible actions.
     * @param costs The list of costs for each action.
     * @param planner The name of the planner being used.
     * @return True if initialization was successful, false otherwise.
     */
    bool init(int **map, int row_size, int col_size, std::vector<std::pair<int, int>> actions, std::vector<double> costs, string planner);

    /**
     * @brief Extracts the path based on the state IDs.
     * @param search The search type (e.g., "frontier", "smgs").
     * @param ids The list of state IDs.
     * @param path The vector to store the extracted path.
     * @return True if the path was successfully extracted, false otherwise.
     */
    bool extractPath(string search, const std::vector<int> &ids, std::vector<GridState *> &path);
};

#endif // GRID_2D_ENVIRONMENT_H_