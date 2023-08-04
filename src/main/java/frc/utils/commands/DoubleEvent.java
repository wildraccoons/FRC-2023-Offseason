package frc.utils.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class provides an easy way to link actions to
 * checked double values.
 * 
 * Triggers can easily be composed for advanced functionality using the 
 * {@link #and(BooleanSupplier)}, {@link #or(BooleanSupplier)}, {@link #negate()}operators.
 * 
 * This class extends {@link Trigger} and implements {@link DoubleEvent}
 * for ease of use with WPILib's Command paradigm.
 */
public class DoubleEvent extends BooleanEvent {
    /** A check to run on a supplied value. */
    public interface Check {
        /**
         * The check to run on a supplied value.
         * @param supplied the supplied value to check.
         * @return whether or not to trigger an event.
         */
        boolean check(double supplied);
    }

    /** 
     * Creates a new event with the given signal and check.
     * @param signal The double signal represented by this object.
     * @param check The check to run on the signal to determine whether or not to trigger an event.
     * @param loop The loop instance that polls this trigger.
     */
    public DoubleEvent(DoubleSupplier signal, Check check, EventLoop loop) {
        super(loop, () -> check.check(signal.getAsDouble()));
    }

    /** 
     * Creates a new event with the given signal and check.
     * Polled by the default scheduler button loop.
     * @param signal The double signal represented by this object.
     * @param check The check to run on the signal to determine whether or not to trigger an event.
     */
    public DoubleEvent(DoubleSupplier signal, Check check) {
        super(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> check.check(signal.getAsDouble()));
    }
}
