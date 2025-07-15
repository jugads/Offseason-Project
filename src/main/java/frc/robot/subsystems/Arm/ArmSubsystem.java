package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmIO.ArmIOInputs;

public class ArmSubsystem extends SubsystemBase {
    private final ArmIO io;
    private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();
    public double armSetpoint;
    public enum WantedState {
        HOLD,
        MOVE_UP,
        MOVE_DOWN,
        GO_TO_REGULAR_SCORE,
        GO_TO_L1,
        GO_TO_L4,
        GO_TO_TRANSFER,
        AUTON_START_MOVE_ARM_OUT
    }

    private enum SystemState {
        HOLDING,
        UPPING,
        DOWNING,
        GOING_TO_SCORE_L2_L3,
        GOING_TO_SCORE_L1,
        GOING_TO_SCORE_L4,
        GOING_TO_TRANSFER,
        MOVING_ARM_OUT
    }
    private WantedState wantedState = WantedState.HOLD;
    private SystemState systemState = SystemState.HOLDING;


    public ArmSubsystem(ArmIO io) {
        this.io = io;
    }

    
    @Override
    public void periodic() {
        SmartDashboard.putString("State", systemState.toString());
        io.updateInputs(inputs);
        io.refreshData();
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            systemState = newState;
        }

        switch (systemState) {
            case UPPING:
                io.setArmSpeed(0.1);
                break;
            case DOWNING:
                io.setArmSpeed(-0.1);
                break;
            case HOLDING:
                io.holdArm();
                break;
            case GOING_TO_SCORE_L2_L3:
                io.setPositionSetpoint(0.175);
                break;
            case GOING_TO_SCORE_L1:
                io.setPositionSetpoint(-0.055);
                break;
            case GOING_TO_SCORE_L4:
                io.setPositionSetpoint(0.1475);
                break;
            case GOING_TO_TRANSFER:
                io.setPositionSetpoint(-0.23);
                break;
            case MOVING_ARM_OUT:
                io.setPositionSetpoint(0.22);
            default:
                io.holdArm();
                break;
        }
    }
    private SystemState handleStateTransition() {
        switch (wantedState) {
            case HOLD:
                return SystemState.HOLDING;
            case GO_TO_REGULAR_SCORE:
                return SystemState.GOING_TO_SCORE_L2_L3;
            case GO_TO_L1:
                return SystemState.GOING_TO_SCORE_L1;
            case GO_TO_L4:
                return SystemState.GOING_TO_SCORE_L4;
            case MOVE_UP:
                return SystemState.UPPING;
            case MOVE_DOWN:
                return SystemState.DOWNING;
            case GO_TO_TRANSFER:
                return SystemState.GOING_TO_TRANSFER;
            case AUTON_START_MOVE_ARM_OUT:
                return SystemState.MOVING_ARM_OUT;
            default:
                return SystemState.HOLDING;
        }
    }
    public void moveUP() {
        setWantedState(WantedState.MOVE_UP);
    }


    public void setWantedState(WantedState state) {
        this.wantedState = state;
    }

    public double getPosition() {
        return inputs.position;
    }
    public WantedState getWantedState() {
        return wantedState;
    }
    public Command setWantedStateCommand(WantedState state) {
        return new InstantCommand(() -> setWantedState(state));
    }
}
