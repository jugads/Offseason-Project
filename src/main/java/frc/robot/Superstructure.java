package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import static frc.robot.Constants.ElevatorConstants.*;

public class Superstructure extends SubsystemBase {
    public enum WantedSuperState {
        IDLE,
        FEED_HOPPER,
        REVERSE_HOPPER,
        HOLD_HOPPER,
        ELEVATOR_UP,
        ELEVATOR_DOWN,
        ELEVATOR_HOLD,
        ELEVATOR_TO_TRANSFER,
        ELEVATOR_TO_L1,
        ELEVATOR_TO_L2,
        ELEVATOR_TO_L3,
        ELEVATOR_TO_L4,
        CLIMBER_UP,
        CLIMBER_DOWN,
        CLIMBER_DEPLOY,
        CLIMB
    }

    public enum CurrentSuperState {
        IDLE,
        FEEDING,
        REVERSING,
        HOLDING,
        RAISING_ELEVATOR,
        LOWERING_ELEVATOR,
        HOLD_ELEVATOR,
        MOVING_ELEVATOR_TO_TRANSFER,
        MOVING_ELEVATOR_TO_L1,
        MOVING_ELEVATOR_TO_L2,
        MOVING_ELEVATOR_TO_L3,
        MOVING_ELEVATOR_TO_L4,
        RAISING_CLIMBER,
        LOWERING_CLIMBER,
        DEPLOYING_CLIMBER,
        CLIMBING
    }

    private final HopperSubsystem hopperSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClimberSubsystem climberSubsystem;

    private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;

    public Superstructure(HopperSubsystem hopper, ElevatorSubsystem elevator, ClimberSubsystem climber) {
        this.hopperSubsystem = hopper;
        this.elevatorSubsystem = elevator;
        this.climberSubsystem = climber;
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStateBehavior();
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case FEED_HOPPER:
                currentSuperState = CurrentSuperState.FEEDING;
                break;
            case REVERSE_HOPPER:
                currentSuperState = CurrentSuperState.REVERSING;
                break;
            case HOLD_HOPPER:
                currentSuperState = CurrentSuperState.HOLDING;
                break;
            case ELEVATOR_UP:
                currentSuperState = CurrentSuperState.RAISING_ELEVATOR;
                break;
            case ELEVATOR_DOWN:
                currentSuperState = CurrentSuperState.LOWERING_ELEVATOR;
                break;
            case ELEVATOR_HOLD:
                currentSuperState = CurrentSuperState.HOLD_ELEVATOR;
                break;
            case ELEVATOR_TO_TRANSFER:
                currentSuperState = CurrentSuperState.MOVING_ELEVATOR_TO_TRANSFER;
                break;
            case ELEVATOR_TO_L1:
                currentSuperState = CurrentSuperState.MOVING_ELEVATOR_TO_L1;
                break;
            case ELEVATOR_TO_L2:
                currentSuperState = CurrentSuperState.MOVING_ELEVATOR_TO_L2;
                break;
            case ELEVATOR_TO_L3:
                currentSuperState = CurrentSuperState.MOVING_ELEVATOR_TO_L3;
                break;
            case ELEVATOR_TO_L4:
                currentSuperState = CurrentSuperState.MOVING_ELEVATOR_TO_L4;
                break;
            case CLIMBER_UP:
                currentSuperState = CurrentSuperState.RAISING_CLIMBER;
                break;
            case CLIMBER_DOWN:
                currentSuperState = CurrentSuperState.LOWERING_CLIMBER;
                break;
            case CLIMBER_DEPLOY:
                currentSuperState = CurrentSuperState.DEPLOYING_CLIMBER;
                break;
            case IDLE:
            default:
                currentSuperState = CurrentSuperState.IDLE;
                break;
        }
    }

    private void applyStateBehavior() {
        switch (currentSuperState) {
            case FEEDING:
                hopperSubsystem.setWantedState(HopperSubsystem.WantedState.FEEDING);
                break;
            case REVERSING:
                hopperSubsystem.setWantedState(HopperSubsystem.WantedState.REVERSE);
                break;
            case HOLDING:
                hopperSubsystem.setWantedState(HopperSubsystem.WantedState.IDLE);
                break;
            case RAISING_ELEVATOR:
                elevatorSubsystem.setManualSpeed(0.3);
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_UP);
                break;
            case LOWERING_ELEVATOR:
                elevatorSubsystem.setManualSpeed(-0.1);
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_DOWN);
                break;
            case HOLD_ELEVATOR:
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD);
                break;
            case MOVING_ELEVATOR_TO_TRANSFER:
                elevatorSubsystem.setSetpointPercent(kElevTran); // mid height
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GO_TO_SETPOINT);
                break;
            case MOVING_ELEVATOR_TO_L1:
                elevatorSubsystem.setSetpointPercent(kElevL1); // mid height
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GO_TO_SETPOINT);
                break;
            case MOVING_ELEVATOR_TO_L2:
                elevatorSubsystem.setSetpointPercent(kElevL2); // mid height
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GO_TO_SETPOINT);
                break;
            case MOVING_ELEVATOR_TO_L3:
                elevatorSubsystem.setSetpointPercent(kElevL3); // mid height
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GO_TO_SETPOINT);
                break;
            case MOVING_ELEVATOR_TO_L4:
                elevatorSubsystem.setSetpointPercent(kElevL4); // mid height
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.GO_TO_SETPOINT);
                break;
            case RAISING_CLIMBER:
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.MOVE_UP);
                break;
            case LOWERING_CLIMBER:
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.MOVE_DOWN);
                break;
            case DEPLOYING_CLIMBER: //!!!!MAKE THIS DEPLOY!!!!!
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.DEPLOY);
            case IDLE:
            default:
                hopperSubsystem.setWantedState(HopperSubsystem.WantedState.IDLE);
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.IDLE);
                break;
        }
    }

    public void setWantedSuperState(WantedSuperState newState) {
        this.wantedSuperState = newState;
    }

    public Command setStateCommand(WantedSuperState newState) {
        return new InstantCommand(() -> {
            System.out.println("Switching to: " + newState);
            setWantedSuperState(newState);
        }, this);
    }
}
