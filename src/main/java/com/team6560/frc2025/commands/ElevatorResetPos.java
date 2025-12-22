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
    
    // Profile constraints for downward movement (VERY slow and safe)
    private static final double DOWN_MAX_VELOCITY = 0.5;      // Very slow speed going down
    private static final double DOWN_MAX_ACCELERATION = 1.0;  // Very gentle acceleration going down
    
    // Profile constraints for upward movement
    private static final double UP_MAX_VELOCITY = 3.0;      // rotations per second
    private static final double UP_MAX_ACCELERATION = 4.0;  // rotations per second²

    public ElevatorResetPos(Elevator elevator) {
        this.elevator = elevator;
        this.timer = new Timer();
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        hasHitLimit = false;
        
        // Create trapezoid profile to move down slowly
        TrapezoidProfile.Constraints downConstraints = new TrapezoidProfile.Constraints(
            DOWN_MAX_VELOCITY,
            DOWN_MAX_ACCELERATION
        );
        
        initialState = new TrapezoidProfile.State(elevator.getElevatorHeight(), 0);
        goalState = new TrapezoidProfile.State(-50, 0); // Move down to well below zero
        
        profile = new TrapezoidProfile(downConstraints);
        timer.restart();
    }

    @Override
    public void execute() {
        if (!hasHitLimit) {
            // Follow trapezoid profile going down
            TrapezoidProfile.State setpoint = profile.calculate(
                timer.get(),
                initialState,
                goalState
            );
            elevator.setElevatorPosition(setpoint.position);
            
            // Check if hit limit switch
            if (elevator.bottomLimitSwitchDown()) {
                hasHitLimit = true;
                elevator.stopMotors();
                elevator.resetEncoderPos(0);
                
                // Now create new profile for going up to STOW
                TrapezoidProfile.Constraints upConstraints = new TrapezoidProfile.Constraints(
                    UP_MAX_VELOCITY, 
                    UP_MAX_ACCELERATION
                );
                initialState = new TrapezoidProfile.State(0, 0);
                goalState = new TrapezoidProfile.State(ElevatorConstants.ElevatorStates.STOW, 0);
                profile = new TrapezoidProfile(upConstraints);
                timer.restart();
            }
        } else {
            // Follow trapezoid profile going up to STOW
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
        } else {
            elevator.stopMotors();
        }
    }
}