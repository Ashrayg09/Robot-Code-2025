package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorResetPos extends Command {
    private final Elevator elevator;
    private boolean hasHitLimit = false;
    
    // Trapezoid profile for smooth movement
    private TrapezoidProfile profile;
    private TrapezoidProfile.State initialState;
    private TrapezoidProfile.State goalState;
    private Timer timer;
    
    // Profile constraints (adjust these for speed)
    private static final double MAX_VELOCITY = 3.0;      // rotations per second
    private static final double MAX_ACCELERATION = 4.0;  // rotations per second²

    public ElevatorResetPos(Elevator elevator) {
        this.elevator = elevator;
        this.timer = new Timer();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        hasHitLimit = false;
        timer.restart();
        // Move down to find limit switch
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
            
            // Create trapezoid profile from 0 to STOW
            TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
                MAX_VELOCITY, 
                MAX_ACCELERATION
            );
            
            initialState = new TrapezoidProfile.State(0, 0);
            goalState = new TrapezoidProfile.State(ElevatorConstants.ElevatorStates.STOW, 0);
            
            profile = new TrapezoidProfile(constraints);
            timer.restart();
        }
        
        // Follow the trapezoid profile after reset
        if (hasHitLimit && profile != null) {
            TrapezoidProfile.State setpoint = profile.calculate(
                timer.get(),
                initialState,
                goalState
            );
            elevator.setElevatorPosition(setpoint.position);
        }
    }

    @Override
    public boolean isFinished() {
        if (!hasHitLimit) {
            return false;
        }
        // Finish when we reach STOW position
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 0.05;
    }

    @Override
    public void end(boolean interrupted) {
        // Hold at STOW position
        if (hasHitLimit && !interrupted) {
            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        }
    }
}