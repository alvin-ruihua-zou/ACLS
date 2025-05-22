#include <unordered_map>
#include <vector>
#include <utility>
#include <bitset>
#include <cstring>
#include "environment.h"

#include <boost/functional/hash.hpp>
using namespace std;

typedef vector<vector<int>> TowerPegs;

/**
 * @struct TowerState
 * @brief Represents a state in the Towers of Hanoi environment.
 */
struct TowerState
{
    TowerPegs pegs;    ///< The current configuration of the pegs.
    TowerPegs bp_pegs; ///< The configuration of the pegs for the parent.
};

/**
 * @brief Equality operator for TowerState.
 *
 * @param a The first TowerState.
 * @param b The second TowerState.
 * @return True if the two TowerStates are equal, false otherwise.
 */
inline bool operator==(const TowerState &a, const TowerState &b)
{
    return a.pegs[0] == b.pegs[0] && a.pegs[1] == b.pegs[1] && a.pegs[2] == b.pegs[2];
}

/**
 * @brief Equality operator for TowerPegs.
 *
 * @param a The first TowerPegs.
 * @param b The second TowerPegs.
 * @return True if the two TowerPegs are equal, false otherwise.
 */
inline bool operator==(const TowerPegs &a, const TowerPegs &b)
{
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

/**
 * @brief Adds two pairs element-wise.
 *
 * @tparam T The type of the first element in the pair.
 * @tparam U The type of the second element in the pair.
 * @param l The first pair.
 * @param r The second pair.
 * @return A new pair with the summed elements.
 */
template <typename T, typename U>
std::pair<T, U> operator+(const std::pair<T, U> &l, const std::pair<T, U> &r)
{
    return {l.first + r.first, l.second + r.second};
}

/**
 * @brief Subtracts two pairs element-wise.
 *
 * @tparam T The type of the first element in the pair.
 * @tparam U The type of the second element in the pair.
 * @param l The first pair.
 * @param r The second pair.
 * @return A new pair with the subtracted elements.
 */
template <typename T, typename U>
std::pair<T, U> operator-(const std::pair<T, U> &l, const std::pair<T, U> &r)
{
    return {l.first - r.first, l.second - r.second};
}

/**
 * @struct PointerValueHash
 * @brief Computes a hash value for a pointer using the hash value of the object it points to.
 * @tparam T The type of the object being pointed to.
 */
template <typename T>
struct PointerValueHash
{
    typedef T *argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type s) const { return std::hash<T>()(*s); }
};

/**
 * @struct PointerValueEqual
 * @brief Tests for equality between two pointers by comparing the objects they point to.
 * @tparam T The type of the object being pointed to.
 */
template <typename T>
struct PointerValueEqual
{
    typedef T *argument_type;
    bool operator()(argument_type a, argument_type b) const { return *a == *b; }
};

/**
 * @namespace std
 * @brief Specialization of the std::hash template for TowerState.
 */
namespace std
{
    template <>
    struct hash<TowerState>
    {
        typedef TowerState argument_type;
        typedef std::size_t result_type;
        /**
         * @brief Computes the hash value for a TowerState.
         * @param s The TowerState to hash.
         * @return The computed hash value.
         */
        result_type operator()(const argument_type &s) const;
    };
}

/**
 * @class AttractorHashFunction
 * @brief Hash function for attractors.
 */
class AttractorHashFunction
{
public:
    /**
     * @brief Computes a hash value for an integer attractor ID.
     * @param p The attractor ID.
     * @return The computed hash value.
     */
    size_t operator()(const int &p) const
    {
        return p;
    }
};

/**
 * @class TowerEnvironment
 * @brief Represents the towers of hanoi environment for path planning.
 */
class TowerEnvironment : public DiscreteSpaceInformation
{
public:
    int m_goal_state_id = -1;  ///< ID of the goal state.
    int m_start_state_id = -1; ///< ID of the start state.
    int m_weight;              ///< Weight for heuristic calculations.
    std::string m_planner;     ///< Name of the planner being used.

    int disc_num; ///< Number of discs in the tower.
    int peg_num;  ///< Number of pegs in the tower.

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

    typedef TowerState StateKey;                                    ///< Alias for the state key type.
    typedef PointerValueHash<StateKey> StateHash;                   ///< Hash function for state keys.
    typedef PointerValueEqual<StateKey> StateEqual;                 ///< Equality comparator for state keys.
    hash_map<StateKey *, int, StateHash, StateEqual> m_state_to_id; ///< Maps state keys to state IDs.

    std::vector<TowerState *> m_states;     ///< Maps state IDs to TowerState pointers.
    std::vector<TowerState *> m_attractors; ///< List of attractor states.

    typedef string StateString; ///< Alias for the state string type.

    typedef unordered_map<int, TowerState *, AttractorHashFunction> frontier_midpoints_t;
    frontier_midpoints_t m_frontier_midpoints;      ///< Maps state IDs to frontier midpoint states.
    frontier_midpoints_t m_temp_frontier_midpoints; ///< Maps state IDs to temporary frontier midpoint states.

    vector<TowerState *> m_ssp; ///< List of sparse solution points.

    std::vector<std::pair<int, int>> m_actions; ///< List of possible actions in the grid.
    std::vector<double> m_costs;                ///< List of costs for each action.

    /**
     * @brief Destructor for the TowerEnvironment class.
     */
    ~TowerEnvironment();

    /**
     * @brief Prints information about a specific state.
     *
     * @param state_id The ID of the state to print.
     */
    void printState(int state_id) override;

    /**
     * @brief Prints information about all attractors.
     */
    void printAttractors() override;

    /**
     * @brief Retrieves the TowerState corresponding to a state ID.
     *
     * @param state_id The ID of the state.
     * @return Pointer to the corresponding TowerState.
     */
    TowerState *getHashEntry(int state_id) const;

    /**
     * @brief Retrieves the state ID for a given TowerPegs.
     *
     * @param pegs The TowerPegs of the state.
     * @return The state ID, or -1 if the state does not exist.
     */
    int getHashEntry(const TowerPegs &pegs);

    /**
     * @brief Creates a new state for a given TowerPegs.
     *
     * @param pegs The TowerPegs of the state.
     * @return The state ID of the newly created state.
     */
    int createHashEntry(const TowerPegs &pegs);

    /**
     * @brief Retrieves or creates a state for a given TowerPegs.
     *
     * @param pegs The TowerPegs of the state.
     * @return The state ID of the retrieved or newly created state.
     */
    int getOrCreateState(const TowerPegs &pegs);

    /**
     * @brief Reserves a new state ID in m_states.
     *
     * @return The reserved state ID.
     */
    int reserveHashEntry();

    /**
     * @brief Sets the goal state of the environment.
     *
     * @param pegs The TowerPegs representing the goal state.
     */
    void setGoalState(const TowerPegs &pegs);

    /**
     * @brief Sets the start state of the environment.
     *
     * @param pegs The TowerPegs representing the start state.
     */
    void setStartState(const TowerPegs &pegs);

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
     * @brief Retrieves the distance to an attractor from a state.
     *
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @return The distance to the attractor.
     */
    double GetAttractorDist(int state_id, int attr_id) override;

    /**
     * @brief Retrieves the distance to an attractor from a TowerPegs instance.
     *
     * @param pegs The TowerPegs of the state.
     * @param attr_id The ID of the attractor.
     * @return The distance to the attractor.
     */
    double GetAttractorDist(const TowerPegs &pegs, int attr_id);

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

    TowerPegs stringToPegs(StateString pegs_string);
    StateString pegsToString(TowerPegs pegs);

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

    /**
     * @brief Recovers the action taken to reach a new state.
     * @param pegs1 The pegs of the first state.
     * @param pegs2 The pegs of the second state.
     * @return A pair representing the action take from pegs1 to pegs2.
     */
    pair<int, int> recoverAction(const TowerPegs &pegs1, const TowerPegs &pegs2);

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
     * @brief Reinitializes SMGS with new start and goal states.
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
     * @brief Initializes the towers of hanoi environments.
     * @param peg_disc_num The number of discs.
     * @param peg_peg_num The number of pegs.
     * @param actions The list of possible actions.
     * @param costs The list of costs for each action.
     * @param planner The name of the planner being used.
     * @return True if initialization was successful, false otherwise.
     */
    bool init(int peg_disc_num, int peg_peg_num, std::vector<std::pair<int, int>> actions, std::vector<double> costs, string planner);

    /**
     * @brief Extracts the path based on the state IDs.
     * @param search The search type (e.g., "frontier", "smgs").
     * @param ids The list of state IDs.
     * @param path The vector to store the extracted path.
     * @return True if the path was successfully extracted, false otherwise.
     */
    bool extractPath(string search, const std::vector<int> &ids, std::vector<TowerState *> &path);
};