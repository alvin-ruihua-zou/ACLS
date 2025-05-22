#ifndef SMGS_PLANNER_H_
#define SMGS_PLANNER_H_

#include <bitset>
#include <queue>
#include <unordered_map>
#include <vector>
#include <chrono>
#include "heap/intrusive_heap.h"
#include "environment.h"

#define INFINITECOST 1000000000

/**
 * @class SMGS
 * @brief Implements the Sparse Multi-Goal Search (SMGS) algorithm for pathfinding.
 */
class SMGS
{
public:
    /**
     * @struct SearchState
     * @brief Represents a state in the search space.
     */
    struct SearchState : public heap_element
    {
        int state_id;          ///< Corresponding graph state ID.
        double g;              ///< Cost-to-come.
        double h;              ///< Estimated cost-to-go (heuristic).
        double f;              ///< Total cost (f = g + h).
        int p;                 ///< In-degree of the state.
        SearchState *ancestor; ///< Pointer to the ancestor state.
    };

    /**
     * @struct SearchStateCompare
     * @brief Comparator for SearchState objects based on their f-values.
     */
    struct SearchStateCompare
    {
        bool operator()(const SearchState &s1, const SearchState &s2) const
        {
            return s1.f < s2.f;
        }
    };

    /**
     * @class HashFunction
     * @brief Hash function for unordered maps.
     */
    class HashFunction
    {
    public:
        size_t operator()(const int &p) const
        {
            return p;
        }
    };

    /**
     * @brief Constructor for the SMGS class.
     * @param space Pointer to the discrete space environment.
     */
    SMGS(DiscreteSpaceInformation *space);

    /**
     * @brief Destructor for the SMGS class.
     */
    ~SMGS();

    /**
     * @brief Prints information about a specific state.
     * @param state_id The ID of the state to print.
     */
    void printState(int state_id);

    /**
     * @brief Computes the f-value for a given state.
     * @param s Pointer to the search state.
     * @return The computed f-value.
     */
    double computeFval(SearchState *s) const;

    /**
     * @brief Retrieves the search state corresponding to a graph state.
     * @param state_id The ID of the graph state.
     * @return Pointer to the corresponding search state.
     */
    SearchState *getSearchState(int state_id);

    /**
     * @brief Creates a new search state for a graph state.
     * @param state_id The ID of the graph state.
     * @return Pointer to the newly created search state.
     */
    SearchState *createState(int state_id);

    /**
     * @brief Initializes a search state.
     * @param state Pointer to the search state to initialize.
     */
    void initSearchState(SearchState *state);

    /**
     * @brief Checks if two floating-point numbers are approximately equal.
     * @param c1 The first number.
     * @param c2 The second number.
     * @return True if the numbers are approximately equal, false otherwise.
     */
    bool almostEqual(double c1, double c2);

    /**
     * @brief Extracts the path from the start state to the goal state.
     * @param start_state Pointer to the start state.
     * @param goal_state Pointer to the goal state.
     * @param solution Vector to store the solution path.
     */
    void extractPath(SearchState *start_state, SearchState *goal_state, std::vector<int> &solution);

    /**
     * @brief Prunes the closed list to reduce memory usage.
     */
    void pruneClosedList();

    /**
     * @brief Extracts a sparse solution path from a given state.
     * @param s Pointer to the state.
     * @return Vector of pointers to the states in the sparse solution path.
     */
    std::vector<SearchState *> extractSparseSolution(SearchState *s);

    /**
     * @brief Plans a path from the start state to the goal state.
     * @param startID The ID of the start state.
     * @param goalID The ID of the goal state.
     * @param solution Pointer to store the solution path.
     * @param cost Pointer to store the total cost of the path.
     * @return Status code indicating success or failure.
     */
    int plan(int startID, int goalID, std::vector<int> *solution, int *cost);

    /**
     * @brief Gets the size of the closed list.
     * @return The size of the closed list.
     */
    int getClosedListSize();

    /**
     * @brief Sets the weight for the heuristic.
     * @param w The weight value.
     */
    inline void setWeight(int w)
    {
        m_weight = w;
    }

    /**
     * @brief Sets the threshold for pruning the closed list.
     * @param t The threshold value.
     */
    inline void setClosedThreshold(int t)
    {
        m_closed_threshold = t;
    }

    /**
     * @brief Sets the maximum threshold for pruning the closed list.
     * @param t The maximum threshold value.
     */
    inline void setClosedMaxThreshold(int t)
    {
        m_closed_max_threshold = t;
    }

    /**
     * @brief Set the timeout for the search.
     * @param t The timeout value in seconds.
     */
    inline void setTimeout(int t)
    {
        m_timeout = t;
    }

    /**
     * @brief Gets the maximum size of the closed list.
     * @return The maximum size of the closed list.
     */
    inline int getClosedMaxSize()
    {
        return m_closed_max_size;
    }

private:
    DiscreteSpaceInformation *m_space;                      ///< Pointer to the discrete space environment.
    intrusive_heap<SearchState, SearchStateCompare> m_open; ///< Open list for SMGS search.
    std::vector<SearchState *> m_states;                    ///< List of all search states.
    std::vector<SearchState *> m_ssp;                       ///< List of sparse solution states.
    typedef std::unordered_map<int, SearchState *, HashFunction> map_t;
    map_t m_midpoint_states;      ///< Map of midpoint states.
    map_t m_temp_midpoint_states; ///< Map of temporary midpoint states.
    map_t m_closed;               ///< Closed list.
    map_t m_dsp;                  ///< Map of dynamic sparse points.
    map_t m_temp_dsp;             ///< Map of temporary dynamic sparse points.

    int m_weight;                  ///< Weight for the heuristic.
    int m_depth;                   ///< Depth of the search.
    int m_start_state_id;          ///< Graph state ID for the start state.
    int m_goal_state_id;           ///< Graph state ID for the goal state.
    int m_init_goal_state;         ///< Initial goal state ID.
    std::vector<int> m_succs;      ///< Vector to store successors when a state is expanded.
    std::vector<double> m_costs;   ///< Vector to store corresponding costs.
    std::vector<int> m_action_ids; ///< Vector to store action IDs when a state is expanded.

    int m_closed_threshold;                                ///< Threshold for pruning the closed list.
    int m_closed_max_threshold;                            ///< Maximum threshold for pruning the closed list.
    int m_closed_max_size;                                 ///< Maximum size of the closed list.
    int m_closed_size;                                     ///< Current size of the closed list.
    std::chrono::high_resolution_clock::time_point t1, t2; ///< Timing variables for performance measurement.
    int m_timeout;                                         ///< Timeout for the search in seconds.
};

#endif // SMGS_PLANNER_H_