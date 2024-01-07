package frc.team1918.lib.statefactory.transition;

// import frc.team1918.lib.statefactory.callbacks.CallbackBase;

/**
 * Holds the boolean condition that determines if a state should transition or not.
 */
@FunctionalInterface
public interface TransitionCondition {
    /**
     * The actual boolean condition.
     * @return Returns the value of the condition (if it has been met or not).
     */
    boolean shouldTransition();
}