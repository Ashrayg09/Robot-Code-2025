package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.Constants.ElevatorConstants;
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
        // Command position well below current to move down
        elevator.setElevatorPosition(-50);
    }

    @Override
    public void execute() {
        // Check if we've hit the bottom limit switch
        if (elevator.bottomLimitSwitchDown() && !hasHitLimit) {
            hasHitLimit = true;
            elevator.stopMotors();
            // Reset encoder to 0 at this position
            elevator.resetEncoderPos(0);
            // Now move to STOW position
            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        }
    }

    @Override
    public boolean isFinished() {
        if (!hasHitLimit) {
            return false;
        }
        // Finish when we've reached STOW position
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        // Position control holds the position automatically
    }
}