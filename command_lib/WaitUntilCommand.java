package org.firstinspires.ftc.teamcode.livewire.command_lib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Supplier;

// A command that delays until a boolean supplier is true
public class WaitUntilCommand extends Command {
    public Supplier<Boolean> condition;
    /**
     * Creates a new WaitUntilCommand. This is a delay command that blocks until a certain boolean supplier reaches true.
     */
    public WaitUntilCommand(Supplier<Boolean> condition) {
        this.condition = condition;
    }

    /**
     * Returns whether or not the sequential command is finished
     * @return Whether the sequential command is finished
     */
    @Override
    public boolean isFinished() {
        return condition.get();
    }
}
