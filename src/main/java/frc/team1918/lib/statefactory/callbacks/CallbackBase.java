package frc.team1918.lib.statefactory.callbacks;

/**
 * The functional interface CallbackBase stores the call info of an action.
 */
@FunctionalInterface
public interface CallbackBase {
    /**
     * The call function stores the actual action.
     */
    void call();
}