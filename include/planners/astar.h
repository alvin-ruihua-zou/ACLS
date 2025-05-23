#ifndef ASTAR_PLANNER_H_
#define ASTAR_PLANNER_H_

#include <bitset>
#include <queue>
#include <unordered_map>
#include <vector>
#include <chrono>
#include "heap/intrusive_heap.h"
#include "environment.h"

#include "compares.h"

#define INFINITECOST 1000000000

/**
 * @class AStar
 * @brief Implements the A* search algorithm for pathfinding in a discrete space.
 */
class AStar
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
        bool closed;                                  ///< Whether the state has been expanded.
        SearchState *bp_state;                        ///< Backpointer to the predecessor state.
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
     * @brief Constructor for the AStar class.
     * @param space Pointer to the discrete space environment.
     */
    AStar(DiscreteSpaceInformation *space);

    /**
     * @brief Destructor for the AStar class.
     */
    ~AStar();

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
     * @brief Retrieves the search state corresponding to a graph state, creating a new state if
     * one has not been created yet.
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
     * @brief Retrieves the path to the goal state.
     * @param goal_state Pointer to the goal state.
     * @return Status code indicating success or failure.
     */
    int getPath(SearchState *goal_state);

    /**
     * @brief Extracts the path from the start state to the goal state.
     * @param start_state Pointer to the start state.
     * @param goal_state Pointer to the goal state.
     * @param solution Vector to store the solution path.
     * @param cost Reference to store the total cost of the path.
     */
    void extractPath(SearchState *start_state, SearchState *goal_state, std::vector<int> &solution, int &cost);

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
     * @brief Retrieves the number of state expansions performed.
     * @return The number of state expansions.
     */
    int getExpansions();

    /**
     * @brief Sets the weight for the heuristic.
     * @param w The weight value.
     */
    inline void setWeight(int w)
    {
        m_weight = w;
    }

    /**
     * @brief Retrieves the number of states in the Open list.
     * @return The number of stated in Open.
     */
    inline int getOpenSize()
    {
        return m_open.size();
    }

private:
    DiscreteSpaceInformation *m_space;                      ///< Pointer to the discrete space environment.
    intrusive_heap<SearchState, SearchStateCompare> m_open; ///< Open list for A* search.
    std::vector<SearchState *> m_states;                    ///< List of all search states.
    int m_weight;                                           ///< Weight for the heuristic.
    int m_start_state_id;                                   ///< ID of the start state.
    int m_goal_state_id;                                    ///< ID of the goal state.
    std::vector<int> m_succs;                               ///< Successor states.
    std::vector<double> m_costs;                            ///< Costs of transitions to successors.
    std::vector<int> m_action_ids;                          ///< Action IDs for transitions to successors.
    int expansions;                                         ///< Number of state expansions.
};

#endif // ASTAR_PLANNER_H_