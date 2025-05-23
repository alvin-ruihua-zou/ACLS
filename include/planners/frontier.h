#ifndef FRONTIER_PLANNER_H_
#define FRONTIER_PLANNER_H_

#include <bitset>
#include <queue>
#include <unordered_map>
#include <vector>
#include <chrono>
#include "heap/intrusive_heap.h"
#include "environment.h"

#define INFINITECOST 1000000000

/**
 * @class FrontierAStar
 * @brief Implements the Frontier A* algorithm for pathfinding.
 */
class FrontierAStar
{
public:
    /**
     * @struct SearchState
     * @brief Represents a state in the search space.
     */
    struct SearchState : public heap_element
    {
        int state_id;                                 ///< Corresponding graph state ID.
        double g;                                     ///< Cost-to-come.
        double h;                                     ///< Estimated cost-to-go (heuristic).
        double f;                                     ///< Total cost (f = g + h).
        SearchState *midpoint_state;                  ///< Pointer to the midpoint state.
        std::bitset<NUM_OF_ACTIONS> action_operators; ///< Available actions for this state.
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
     * @brief Constructor for the FrontierAStar class.
     * @param space Pointer to the discrete space environment.
     */
    FrontierAStar(DiscreteSpaceInformation *space);

    /**
     * @brief Destructor for the FrontierAStar class.
     */
    ~FrontierAStar();

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
     * @brief Expands a search state, updating its successors.
     * @param s Pointer to the search state to expand.
     */
    void expand(SearchState *s);

    /**
     * @brief Retrieves the cost of the path to the goal state.
     * @param goal_state Pointer to the goal state.
     * @param cost Pointer to store the total cost of the path.
     * @return Status code indicating success or failure.
     */
    int getPathCost(SearchState *goal_state, int *cost);

    /**
     * @brief Extracts the path from the start state to the goal state.
     * @param start_state Pointer to the start state.
     * @param goal_state Pointer to the goal state.
     * @param midline_cost The cost of the midpoint.
     * @param solution Vector to store the solution path.
     * @param cost Reference to store the total cost of the path.
     * @param depth The depth of the recursion.
     */
    void extractPath(SearchState *start_state, SearchState *goal_state, int midline_cost, std::vector<int> &solution, int &cost, int depth);

    /**
     * @brief Retrieves the midpoint state between the start and goal states.
     * @param start_state Pointer to the start state.
     * @param goal_state Pointer to the goal state.
     * @param midline_cost The cost of the midpoint.
     * @param goal_cost Pointer to store the cost of the goal state.
     * @return Pointer to the midpoint state.
     */
    SearchState *getMidpointState(SearchState *start_state, SearchState *goal_state, int midline_cost, int *goal_cost);

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
     * @brief Sets the weight for the heuristic.
     * @param w The weight value.
     */
    inline void setWeight(int w)
    {
        m_weight = w;
    }

private:
    DiscreteSpaceInformation *m_space;                      ///< Pointer to the discrete space environment.
    intrusive_heap<SearchState, SearchStateCompare> m_open; ///< Open list for Frontier A* search.
    std::vector<SearchState *> m_states;                    ///< List of all search states.
    typedef std::unordered_map<int, SearchState *, HashFunction> frontier_midpoints_t;
    frontier_midpoints_t m_midpoint_states;      ///< Map of midpoint states.
    frontier_midpoints_t m_temp_midpoint_states; ///< Map of temporary midpoint states.

    int m_weight;                  ///< Weight for the heuristic.
    int expansions;                ///< Number of state expansions.
    int m_start_state_id;          ///< Graph state ID for the start state.
    int m_goal_state_id;           ///< Graph state ID for the goal state.
    int m_init_goal_state;         ///< Initial goal state ID.
    std::vector<int> m_succs;      ///< Vector to store successors when a state is expanded.
    std::vector<double> m_costs;   ///< Vector to store corresponding costs.
    std::vector<int> m_action_ids; ///< Vector to store action IDs when a state is expanded.
};

#endif // FRONTIER_PLANNER_H_