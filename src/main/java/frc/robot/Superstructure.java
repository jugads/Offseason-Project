package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Hopper.HopperSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Bluetooth.BluetoothSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.WantedState;
import static frc.robot.Constants.ElevatorConstants.*;

public class Superstructure extends SubsystemBase {
    public enum WantedSuperState {
        IDLE,
        L1,
        L2,
        L3,
        L4,
        AL2,
        AL3,
        CLIMBER_UP,
        CLIMBER_DOWN,
        CLIMBER_DEPLOY,
        CLIMB,
        TRANSFER_PREP,
        HANDOFF,
        SWINGLY_DINGLY,
        SCORE
    }

    public enum CurrentSuperState {
        IDLE,
        TO_L1,
        TO_L2,
        TO_L3,
        TO_L4,
        TO_AL2,
        TO_AL3,
        RAISING_CLIMBER,
        LOWERING_CLIMBER,
        DEPLOYING_CLIMBER,
        CLIMBING,
        TRANSFER_PREPPING,
        HANDING_OFF,
        SWINGINGLY_DINGINGLY,
        SCORING
    }

    private final HopperSubsystem hopperSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final BluetoothSubsystem bluetoothSubsystem;
    private final ArmSubsystem armSubsystem;
    private WantedSuperState wantedSuperState = WantedSuperState.IDLE;
    private CurrentSuperState currentSuperState = CurrentSuperState.IDLE;
    boolean elevatorCanMove = false;
    public Superstructure(HopperSubsystem hopper, ElevatorSubsystem elevator, ClimberSubsystem climber, BluetoothSubsystem bluetooth, ArmSubsystem arm) {
        this.hopperSubsystem = hopper;
        this.elevatorSubsystem = elevator;
        this.climberSubsystem = climber;
        this.bluetoothSubsystem = bluetooth;
        this.armSubsystem = arm;
    }

    @Override
    public void periodic() {
        handleStateTransitions();
        applyStateBehavior();
        if (armSubsystem.getPosition() < -0.07 || armSubsystem.getPosition() > 0.26) {
            elevatorCanMove = false;
        }
        else {
            elevatorCanMove = true;
        }
        SmartDashboard.putBoolean("Superstructure/Elevator Can Move", elevatorCanMove);
    }

    private void handleStateTransitions() {
        switch (wantedSuperState) {
            case L1:
                currentSuperState = CurrentSuperState.TO_L1;
                break;
            case L2:
                currentSuperState = CurrentSuperState.TO_L2;
                break;
            case L3:
                currentSuperState = CurrentSuperState.TO_L3;
                break;
            case L4:
                currentSuperState = CurrentSuperState.TO_L4;
                break;
            case AL2:
                currentSuperState = CurrentSuperState.TO_AL2;
                break;
            case AL3:
                currentSuperState = CurrentSuperState.TO_AL3;
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
            case TRANSFER_PREP:
                currentSuperState = CurrentSuperState.TRANSFER_PREPPING;
                break;
            case HANDOFF:
                currentSuperState = CurrentSuperState.HANDING_OFF;
                break;
            case SWINGLY_DINGLY:
                currentSuperState = CurrentSuperState.SWINGINGLY_DINGINGLY;
                break;
            case SCORE:
                currentSuperState = CurrentSuperState.SCORING;
                break;
            case IDLE:
            default:
                currentSuperState = CurrentSuperState.IDLE;
                break;
        }
    }

    private void applyStateBehavior() {
        switch (currentSuperState) {
            case TO_L1:
                if (elevatorCanMove) {
                    elevatorSubsystem.setSetpointPercent(kElevL1);
                    elevatorSubsystem.setWantedState(WantedState.GO_TO_SETPOINT);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_L1);
                }
                else {
                    elevatorSubsystem.setWantedState(WantedState.HOLD);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_L1);
                }
                break;
            case TO_L2:
                if (elevatorCanMove) {
                    elevatorSubsystem.setSetpointPercent(kElevL2);
                    elevatorSubsystem.setWantedState(WantedState.GO_TO_SETPOINT);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE);
                }
                else {
                    elevatorSubsystem.setWantedState(WantedState.HOLD);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE);
                }
                break;
            case TO_L3:
                if (elevatorCanMove) {
                    elevatorSubsystem.setSetpointPercent(kElevL3);
                    elevatorSubsystem.setWantedState(WantedState.GO_TO_SETPOINT);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE);
                }
                else {
                    elevatorSubsystem.setWantedState(WantedState.HOLD);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE);
                }
                break;
            case TO_L4:
                if (elevatorCanMove) {
                    elevatorSubsystem.setSetpointPercent(kElevL4);
                    elevatorSubsystem.setWantedState(WantedState.GO_TO_SETPOINT);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_L4);
                }
                else {
                    elevatorSubsystem.setWantedState(WantedState.HOLD);
                    armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_L4);
                }
                break;
            case RAISING_CLIMBER:
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.MOVE_UP);
                break;
            case LOWERING_CLIMBER:
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.MOVE_DOWN);
                break;
            case DEPLOYING_CLIMBER: //!!!!MAKE THIS DEPLOY!!!!!
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.DEPLOY);
                break;
            case TRANSFER_PREPPING:
                elevatorSubsystem.setSetpointPercent(kElevTran);
                if (elevatorSubsystem.atPosition()) {
                    armSubsystem.setWantedState(frc.robot.subsystems.Arm.ArmSubsystem.WantedState.GO_TO_TRANSFER);
                    elevatorSubsystem.setWantedState(WantedState.HOLD);
                }
                else {
                    System.out.println("Not Moving arm");
                    elevatorSubsystem.setWantedState(WantedState.GO_TO_SETPOINT);
                }
                break;
            case HANDING_OFF:
                bluetoothSubsystem.setWantedState(BluetoothSubsystem.WantedState.SUCKING);
                hopperSubsystem.setWantedState(HopperSubsystem.WantedState.FEEDING);
                break;
            case SWINGINGLY_DINGINGLY:
                armSubsystem.setWantedState(ArmSubsystem.WantedState.GO_TO_REGULAR_SCORE);
                elevatorSubsystem.setSetpointPercent(kElevTran+0.09);
                elevatorSubsystem.setWantedState(WantedState.GO_TO_SETPOINT);
                break;
            case SCORING:
                bluetoothSubsystem.setWantedState(BluetoothSubsystem.WantedState.SCORING);
                break;
            case IDLE:
            default:
                hopperSubsystem.setWantedState(HopperSubsystem.WantedState.IDLE);
                elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOLD);
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.IDLE);
                armSubsystem.setWantedState(ArmSubsystem.WantedState.HOLD);
                bluetoothSubsystem.setWantedState(BluetoothSubsystem.WantedState.IDLE);
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
