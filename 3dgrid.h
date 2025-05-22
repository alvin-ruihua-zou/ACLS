#include <unordered_map>
#include <vector>
#include <utility>
#include <bitset>
#include <cstring>
#include <algorithm>
#include <functional>
#include <tuple>
#include "environment.h"

#include <boost/functional/hash.hpp>
using namespace std;

typedef tuple<int, int, int> Robot3DCoord;

/**
 * @struct GridState
 * @brief Represents a state in the 3D grid environment.
 */
struct GridState
{
    int x, y, z;          ///< Coordinates of the state in the 3D grid.
    int bp_x, bp_y, bp_z; ///< Coordinates of the parent of the state.
};

/**
 * @brief Equality operator for GridState.
 * @param a The first GridState.
 * @param b The second GridState.
 * @return True if the two states are equal, false otherwise.
 */
inline bool operator==(const GridState &a, const GridState &b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

/**
 * @namespace internal
 * @brief Contains helper functions for tuple operations.
 */
namespace internal
{
    /**
     * @brief Adds the elements of the second tuple to the first tuple.
     * @tparam T The tuple type.
     * @tparam Is The indices of the tuple elements.
     * @param t1 The first tuple (modified in place).
     * @param t2 The second tuple.
     */
    template <typename T, size_t... Is>
    void add_rhs_to_lhs(T &t1, const T &t2, std::integer_sequence<size_t, Is...>)
    {
        auto l = {(std::get<Is>(t1) += std::get<Is>(t2), 0)...};
        (void)l; // Prevent unused variable warning.
    }

    /**
     * @brief Subtracts the elements of the second tuple from the first tuple.
     * @tparam T The tuple type.
     * @tparam Is The indices of the tuple elements.
     * @param t1 The first tuple (modified in place).
     * @param t2 The second tuple.
     */
    template <typename T, size_t... Is>
    void minus_rhs_to_lhs(T &t1, const T &t2, std::integer_sequence<size_t, Is...>)
    {
        auto l = {(std::get<Is>(t1) -= std::get<Is>(t2), 0)...};
        (void)l; // Prevent unused variable warning.
    }
}

/**
 * @brief Adds the elements of two tuples.
 * @tparam T The tuple types.
 * @param lhs The first tuple.
 * @param rhs The second tuple.
 * @return A new tuple with the summed elements.
 */
template <typename... T>
std::tuple<T...> operator+(std::tuple<T...> lhs, const std::tuple<T...> &rhs)
{
    internal::add_rhs_to_lhs(lhs, rhs, std::index_sequence_for<T...>{});
    return lhs;
}

/**
 * @brief Subtracts the elements of two tuples.
 * @tparam T The tuple types.
 * @param lhs The first tuple.
 * @param rhs The second tuple.
 * @return A new tuple with the subtracted elements.
 */
template <typename... T>
std::tuple<T...> operator-(std::tuple<T...> lhs, const std::tuple<T...> &rhs)
{
    internal::minus_rhs_to_lhs(lhs, rhs, std::index_sequence_for<T...>{});
    return lhs;
}

/**
 * @brief Adds the elements of two vectors element-wise.
 * @tparam T The vector element type.
 * @param a The first vector.
 * @param b The second vector.
 * @return A new vector with the summed elements.
 */
template <typename T>
std::vector<T> operator+(const std::vector<T> &a, const std::vector<T> &b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(),
                   std::back_inserter(result), std::plus<T>());
    return result;
}

/**
 * @brief Subtracts the elements of two vectors element-wise.
 * @tparam T The vector element type.
 * @param a The first vector.
 * @param b The second vector.
 * @return A new vector with the subtracted elements.
 */
template <typename T>
std::vector<T> operator-(const std::vector<T> &a, const std::vector<T> &b)
{
    assert(a.size() == b.size());

    std::vector<T> result;
    result.reserve(a.size());

    std::transform(a.begin(), a.end(), b.begin(),
                   std::back_inserter(result), std::minus<T>());
    return result;
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
        std::size_t seed = 0;
        boost::hash_combine(seed, state->x);
        boost::hash_combine(seed, state->y);
        boost::hash_combine(seed, state->z);
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
        return lhs->x == rhs->x && lhs->y == rhs->y && lhs->z == rhs->z;
    }
};

/**
 * @namespace std
 * @brief Specialization of the std::hash template for GridState.
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
 * @class Grid3DEnvironment
 * @brief Represents a 3D grid environment for path planning.
 *
 * This class provides methods for managing states, actions, and heuristics
 * in a 3D grid environment. It supports operations such as creating states,
 * retrieving successors, and managing attractors.
 */
class Grid3DEnvironment : public DiscreteSpaceInformation
{
public:
    int ***stateMap;     ///< 3D array representing the grid map.
    int map_row_size;    ///< Number of rows in the grid.
    int map_col_size;    ///< Number of columns in the grid.
    int map_height_size; ///< Height of the grid.

    int m_goal_state_id = -1;  ///< ID of the goal state.
    int m_start_state_id = -1; ///< ID of the start state.

    /**
     * @brief Hash map type for mapping state keys to state IDs.
     */
    template <
        class Key,
        class T,
        class Hash = std::hash<Key>,
        class KeyEqual = std::equal_to<Key>,
        class Allocator = std::allocator<std::pair<const Key, T>>>
    using hash_map = std::unordered_map<Key, T, Hash, KeyEqual, Allocator>;

    typedef GridState StateKey;                                     ///< Alias for the state key type.
    hash_map<StateKey *, int, StateHash, StateEqual> m_state_to_id; ///< Maps GridState pointers to state IDs.

    std::string m_planner; ///< Name of the planner being used.

    std::vector<GridState *> m_states;     ///< Maps state IDs to GridState pointers.
    std::vector<GridState *> m_attractors; ///< List of attractor states.

    typedef unordered_map<int, GridState *> frontier_midpoints_t;
    frontier_midpoints_t m_frontier_midpoints;      ///< Maps state IDs to frontier midpoint states.
    frontier_midpoints_t m_temp_frontier_midpoints; ///< Maps state IDs to temporary frontier midpoint states.

    std::vector<GridState *> m_ssp; ///< List of sparse solution points.

    std::vector<std::tuple<int, int, int>> m_actions; ///< List of possible actions in the 3D grid.
    std::vector<double> m_costs;                      ///< List of costs for each action.
    int m_weight;                                     ///< Weight for heuristic calculations.

    /**
     * @brief Destructor for the Grid3DEnvironment class.
     */
    ~Grid3DEnvironment();

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
     * @brief Retrieves the state ID for a given 3D coordinate.
     * @param x The x-coordinate of the state.
     * @param y The y-coordinate of the state.
     * @param z The z-coordinate of the state.
     * @return The state ID, or -1 if the state does not exist.
     */
    int getHashEntry(int x, int y, int z);

    /**
     * @brief Creates a new state for a given 3D coordinate.
     * @param x The x-coordinate of the state.
     * @param y The y-coordinate of the state.
     * @param z The z-coordinate of the state.
     * @return The state ID of the newly created state.
     */
    int createHashEntry(int x, int y, int z);

    /**
     * @brief Retrieves or creates a state for a given 3D coordinate.
     * @param x The x-coordinate of the state.
     * @param y The y-coordinate of the state.
     * @param z The z-coordinate of the state.
     * @return The state ID of the retrieved or newly created state.
     */
    int getOrCreateState(int x, int y, int z);

    /**
     * @brief Reserves a new state ID in m_states.
     * @return The reserved state ID.
     */
    int reserveHashEntry();

    /**
     * @brief Sets the goal state in the 3D grid environment.
     *
     * @param coord The 3D coordinates of the goal state.
     */
    void setGoalState(const Robot3DCoord &coord);

    /**
     * @brief Sets the start state in the 3D grid environment.
     *
     * @param coord The 3D coordinates of the start state.
     */
    void setStartState(const Robot3DCoord &coord);

    /**
     * @brief Retrieves the state ID of the start state.
     *
     * @return The state ID of the start state.
     */
    int getStartStateID();

    /**
     * @brief Retrieves the state ID of the goal state.
     *
     * @return The state ID of the goal state.
     */
    int getGoalStateID();

    /**
     * @brief Checks if a 3D coordinate is free (not an obstacle).
     *
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     * @param z The z-coordinate.
     * @return True if the coordinate is free, false otherwise.
     */
    bool coordFree(int x, int y, int z);

    /**
     * @brief Retrieves the distance to an attractor from a state.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @return The distance to the attractor.
     */
    double GetAttractorDist(int state_id, int attr_id) override;

    /**
     * @brief Retrieves the distance to an attractor from a 3D coordinate.
     *
     * @param x The x-coordinate.
     * @param y The y-coordinate.
     * @param z The z-coordinate.
     * @param attr_id The ID of the attractor.
     * @return The distance to the attractor.
     */
    double GetAttractorDist(int x, int y, int z, int attr_id);

    /**
     * @brief Creates an attractor for a given state.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     */
    void createAttractor(int state_id, int attr_id) override;

    /**
     * @brief Removes an attractor from the environment.
     *
     * @param state_id The ID of the attractor to remove.
     */
    void removeAttractor(int state_id) override;

    /**
     * @brief Checks if a state matches an attractor.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @return True if the state matches the attractor, false otherwise.
     */
    bool sameState(int state_id, int attr_id) override;

    /**
     * @brief Checks if two states are neighbors in the 3D grid.
     *
     * @param search The type of search being performed.
     * @param start_state_id The ID of the start state.
     * @param goal_state_id The ID of the goal state.
     * @return True if the states are neighbors, false otherwise.
     */
    bool isNeighbor(string search, int start_state_id, int goal_state_id) override;

    /**
     * @brief Computes the heuristic estimate to the goal state.
     *
     * @param state_id The ID of the state.
     * @return The heuristic estimate to the goal state.
     */
    double GetGoalHeuristic(int state_id) override;

    /**
     * @brief Retrieves the successors of a state with action operators.
     *
     * @param state_id The ID of the state.
     * @param action_operators A bitset representing the available actions.
     * @param succs Vector to store successor state IDs.
     * @param costs Vector to store the costs of transitions to successors.
     * @param actions Vector to store the action IDs leading to successors.
     */
    void GetSuccs(int state_id, bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions) override;

    /**
     * @brief Retrieves the successors of a state without action operators.
     *
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
     * @brief Initializes the 3D grid environment with a map and actions.
     *
     * @param map The 3D grid map.
     * @param row_size The number of rows in the grid.
     * @param col_size The number of columns in the grid.
     * @param height_size The height of the grid.
     * @param actions The list of possible actions in the grid.
     * @param costs The list of costs for each action.
     * @param planner The name of the planner being used.
     * @return True if initialization was successful, false otherwise.
     */
    bool init(int ***map, int row_size, int col_size, int height_size, std::vector<std::tuple<int, int, int>> actions, std::vector<double> costs, string planner);

    /**
     * @brief Extracts a path from a list of state IDs.
     *
     * @param search The type of search being performed.
     * @param ids The list of state IDs in the path.
     * @param path Vector to store the extracted path as GridState pointers.
     * @return True if the path was successfully extracted, false otherwise.
     */
    bool extractPath(string search, const std::vector<int> &ids, std::vector<GridState *> &path);
};