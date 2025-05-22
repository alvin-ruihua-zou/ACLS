#ifndef ENVIRONMENT_H_
#define ENVIRONMENT_H_

#include <bitset>
#include <vector>
#include <string>
#include <cmath>

#define NUM_OF_ACTIONS 4

/**
 * @class DiscreteSpaceInformation
 * @brief Abstract class representing a discrete space environment for planning.
 *
 * This class provides an interface for managing states, actions, and heuristics
 * in a discrete space environment. It is designed to be extended for specific
 * implementations of planning environments.
 */
class DiscreteSpaceInformation
{
public:
    /**
     * @brief Computes the heuristic estimate from a state to the goal state.
     * @param stateID The ID of the state.
     * @return The heuristic estimate to the goal state.
     */
    virtual double GetGoalHeuristic(int stateID) = 0;

    /**
     * @brief Prints information about a specific state.
     * @param stateID The ID of the state to print.
     */
    virtual void printState(int stateID) = 0;

    /**
     * @brief Prints information about attractors in the environment.
     */
    virtual void printAttractors() = 0;

    /**
     * @brief Gets the successors of a state.
     * @param SourceStateID The ID of the source state.
     * @param SuccIDV Vector to store successor state IDs.
     * @param CostV Vector to store the costs of transitions to successors.
     * @param ActionsIDV Vector to store the action IDs leading to successors.
     */
    virtual void GetSuccs(int SourceStateID, std::vector<int> *SuccIDV, std::vector<double> *CostV, std::vector<int> *ActionsIDV) = 0;

    /**
     * @brief Gets the successors of a state with action operators. (For planners without a closed list)
     * @param SourceStateID The ID of the source state.
     * @param action_operators A bitset representing the available actions.
     * @param SuccIDV Vector to store successor state IDs.
     * @param CostV Vector to store the costs of transitions to successors.
     * @param ActionsIDV Vector to store the action IDs leading to successors.
     */
    virtual void GetSuccs(int SourceStateID, std::bitset<NUM_OF_ACTIONS> action_operators, std::vector<int> *SuccIDV, std::vector<double> *CostV, std::vector<int> *ActionsIDV) = 0;

    /**
     * @brief Gets successors and the best predecessor for a state. (For solution reconstruction)
     * @param state_id The ID of the state.
     * @param action_operators A bitset representing the available actions.
     * @param attr_id The ID of the attractor.
     * @param bp_idx The index of the best predecessor.
     * @param best_pred_match Pointer to a boolean indicating if the best predecessor matches.
     * @param matched_idx Pointer to the index of the matched predecessor.
     * @param succs Vector to store successor state IDs.
     * @param costs Vector to store the costs of transitions to successors.
     * @param actions Vector to store the action IDs leading to successors.
     */
    virtual void GetSuccsAndBestPred(int state_id, std::bitset<NUM_OF_ACTIONS> action_operators, int attr_id, int bp_idx, bool *best_pred_match, std::vector<int> *succs, std::vector<double> *costs, std::vector<int> *actions) = 0;

    /**
     * @brief Gets the best predecessor for a state.
     * @param SourceStateID The ID of the source state.
     * @param attrID The ID of the attractor.
     * @return The ID of the best predecessor state.
     */
    virtual int GetBestPred(int SourceStateID, int attrID) = 0;

    /**
     * @brief Gets the best predecessor for a state, creating it if it does not exist.
     * @param SourceStateID The ID of the source state.
     * @param attrID The ID of the attractor.
     * @return The ID of the best predecessor state.
     */
    virtual int GetBestPredWithCreate(int SourceStateID, int attrID) = 0;

    /**
     * @brief Checks if two states are the same.
     * @param state_id The ID of the first state.
     * @param attr_id The ID of the second state.
     * @return True if the states are the same, false otherwise.
     */
    virtual bool sameState(int state_id, int attr_id) = 0;

    /**
     * @brief Checks if two states are neighbors.
     * @param search The search algorithm (e.g., "A*", "frontier", "smgs").
     * @param start_state_id The ID of the start state.
     * @param goal_state_id The ID of the goal state.
     * @return True if the states are neighbors, false otherwise.
     */
    virtual bool isNeighbor(std::string search, int start_state_id, int goal_state_id) = 0;

    /**
     * @brief Get the matching predecessor state after breaking ties.
     * @param state_id The ID of the state.
     * @param attr_id The ID of the attractor.
     * @param bp_id The ID of the
     */
    virtual int getMatchingState(int state_id, int attr_id, int bp_id, int bp2_id, int bp2_attr_id) = 0;
    virtual void UpdateBP(int state_id, int bp_state_id) = 0;

    /**
     * @brief Deletes a state from the environment.
     * @param stateID The ID of the state to delete.
     */
    virtual void DeleteState(int stateID) = 0;

    /**
     * @brief Creates an attractor for a state.
     * @param stateID The ID of the state.
     * @param attr_id The ID of the attractor.
     */
    virtual void createAttractor(int stateID, int attr_id) = 0;

    /**
     * @brief Removes an attractor from the environment.
     * @param state_id The ID of the state associated with the attractor.
     */
    virtual void removeAttractor(int state_id) = 0;

    /**
     * @brief Gets the distance to an attractor.
     * @param stateID The ID of the state.
     * @param attrID The ID of the attractor.
     * @return The distance to the attractor.
     */
    virtual double GetAttractorDist(int stateID, int attrID) = 0;

    /**
     * @brief Adds a midpoint state to the environment.
     * @param stateID The ID of the state.
     * @param newID The ID of the new midpoint state.
     */
    virtual void AddMidpointState(int stateID, int newID) = 0;

    virtual void AddTempMidpointState(int stateID) = 0;

    /**
     * @brief Clears temporary midpoint states.
     */
    virtual void ClearTempMidpoints() = 0;

    /**
     * @brief Reinitializes the frontier search.
     * @param startStateID The ID of the start state.
     * @param goalStateID The ID of the goal state.
     */
    virtual void reinit_frontier_search(int startStateID, int goalStateID) = 0;

    /**
     * @brief Get the number of successors a state has.
     * @param state_id The ID of the state.
     * @return The number of successors.
     */
    virtual int getInDegree(int state_id) = 0;

    /**
     * @brief Add state to the sparse solution path.
     * @param state_id The ID of the state to add.
     */
    virtual void addSspState(int state_id) = 0;

    /**
     * @brief Reinitializes variables for the SMGS search.
     */
    virtual void reinit_smgs(int startStateID, int goalStateID, int start_ssp_idx, int goal_ssp_idx) = 0;

    /**
     * @brief Gets the number of states in the environment.
     * @return The number of states.
     */
    virtual int getStateNum() = 0;
};

#endif // ENVIRONMENT_H_