package frc.team1918.lib.statefactory.state;

import frc.team1918.lib.statefactory.callbacks.CallbackBase;
import frc.team1918.lib.statefactory.transition.TransitionCondition;

import java.util.List;

/**
 * Represents each state and all of its properties
 */
@SuppressWarnings("rawtypes")
public class State {
    private Enum name; // taking in the enum constant
    private CallbackBase enterActions;
    private CallbackBase exitActions;
    private CallbackBase loopActions;
    private List<Triple<TransitionCondition, Enum, CallbackBase>> transitions; // Pair: [TransitionCondition, Enum]
    private boolean isFailsafe;

    /**
     * Construct the state in the most basic form.
     * @param name The enum that is to be the name of the state.
     * @param enterActions The callback that will be executed on entering the state.
     * @param loopActions Actions that will constantly be executed during the state.
     * @param exitActions The callback that will be executed on exiting the state.
     * @param transitions The array holding all the transitions from this state. Made up of triples that hold the condition, pointer (if one is specified), and override exit action (if one is specified).
     */
    public State(Enum name, CallbackBase enterActions, CallbackBase loopActions, CallbackBase exitActions, List<Triple<TransitionCondition, Enum, CallbackBase>> transitions) {
        this.name = name;
        this.enterActions = enterActions;
        this.exitActions = exitActions;
        this.transitions = transitions;
        this.loopActions = loopActions;
        isFailsafe = false;
    }

    /**
     * Construct the state with a specified boolean to tell if the state will be handled as a fallback.
     * @param name The enum that is to be the name of the state.
     * @param enterActions The callback that will be executed on entering the state.
     * @param loopActions Actions that will constantly be executed during the state.
     * @param exitActions The callback that will be executed on exiting the state.
     * @param transitions The array holding all the transitions that are to be assigned to this state. Made up of triples that hold the condition, pointer (if one is specified), and override exit action (if one is specified).
     * @param isFailsafe Specifies if this state should be handled as a fallback state or a normal state.
     */
    public State(Enum name, CallbackBase enterActions, CallbackBase loopActions, CallbackBase exitActions, List<Triple<TransitionCondition, Enum, CallbackBase>> transitions, boolean isFailsafe) {
        this.name = name;
        this.enterActions = enterActions;
        this.exitActions = exitActions;
        this.transitions = transitions;
        this.loopActions = loopActions;
        this.isFailsafe = false;
    }

    /**
     * Gets the name of the state.
     * @return Returns the of the state in enum form.
     */
    public Enum getName() {
        return name;
    }

    /**
     * Gets the name of the state.
     * @return Returns the of the state in string form.
     */
    public String getNameAsString() {
        return name.toString();
    }

    /**
     * Gets the CallbackBase containing the state enter actions.
     * @return Returns the callback containing the state enter actions.
     */
    public CallbackBase getEnterActions() {
        return enterActions;
    }

    /**
     * Sets the state's enter action.
     * @param actions The CallbackBase that is to be the states new enterAction.
     */
    public void setEnterActions(CallbackBase actions) {
        enterActions = actions;
    }

    /**
     * Gets the transition array.
     * @return Returns the array holding all the transitions that are to be assigned to this state. Made up of triples that hold the condition, pointer (if one is specified), and override exit action (if one is specified).
     */
    public List<Triple<TransitionCondition, Enum, CallbackBase>> getTransitions() {
        return transitions;
    }

    /**
     * Gets the loop actions.
     * @return Returns the callback containing the states loop actions.
     */
    public CallbackBase getLoopActions() {
        return loopActions;
    }

    /**
     * Sets the loop actions.
     * @param actions The CallBackBase that will be set as the new loop action.
     */
    public void setLoopActions(CallbackBase actions) {
        loopActions = actions;
    }

    /**
     * Get the boolean that specifies if the state is a fallback or not.
     * @return Returns the valus of isFailsafe.
     */
    public boolean isFailsafe() {
        return isFailsafe;
    }

    /**
     * Gets the CallbackBase containing the state's exit actions.
     * @return Returns the callback containing the state's exit actions.
     */
    public CallbackBase getExitActions() {
        return exitActions;
    }

    /**
     * Sets the state's exit action.
     * @param exitActions The CallbackBase that is to be the states new exitAction.
     */
    public void setExitActions(CallbackBase exitActions) {
        this.exitActions = exitActions;
    }
}