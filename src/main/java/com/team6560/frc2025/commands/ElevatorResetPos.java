package com.team6560.frc2025.commands;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.controls.XboxControls;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Elevator.State;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorResetPos extends Command {
    private final Elevator elevator;
    private boolean hasHitLimit = false;

    public ElevatorResetPos(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        hasHitLimit = false;
        // Move down to find bottom limit switch
        elevator.setElevatorPosition(-50);
    }

    @Override
    public void execute() {
        // Check if we've hit the bottom limit switch
        if (elevator.bottomLimitSwitchDown() && !hasHitLimit) {
            hasHitLimit = true;
            // Reset encoder to 0 at this position
            elevator.resetEncoderPos(0);
            // Now move to the stow/reset position
            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        }
    }

    @Override
    public boolean isFinished() {
        // Finish when we've hit the limit and reached the target position
        if (!hasHitLimit) {
            return false;
        }
        // Check if close to STOW position (within 1 rotation tolerance)
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        // Position control holds the position automatically
    }
}