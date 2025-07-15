package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Superstructure.WantedSuperState;
import frc.robot.commands.TransferCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem.WantedState;
import frc.robot.subsystems.Bluetooth.BluetoothSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Hopper.HopperSubsystem;

import static frc.robot.Constants.AlignmentPoses.kAliBLUE2_3;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE4_5;
import static frc.robot.Constants.AutoPoses.*;
import static frc.robot.Constants.ReefPoses.*;

public class AutoRoutines {
    private final AutoFactory m_factory;
    Superstructure superstructure;
    CommandSwerveDrivetrain drivetrain;
    ArmSubsystem arm;
    ElevatorSubsystem elevator;
    BluetoothSubsystem bluetooth;
    HopperSubsystem hopper;
    
    


    public AutoRoutines(AutoFactory factory, Superstructure superstructure, CommandSwerveDrivetrain drivetrain, ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, BluetoothSubsystem bluetoothSubsystem, HopperSubsystem hopperSubsystem) {
        m_factory = factory;
        this.superstructure = superstructure;
        this.drivetrain = drivetrain;
        this.arm = armSubsystem;
        this.elevator = elevatorSubsystem;
        this.bluetooth = bluetoothSubsystem;
        this.hopper = hopperSubsystem;
    }

    public Command DriveTest() {
        var initialPose = getRightSideAutoStartingPose();
        return Commands.sequence(
            new InstantCommand(() -> drivetrain.resetPose(initialPose)),
            m_factory.trajectoryCmd("StartToF")

        );
    }
    public Command BranchesFCD() {
        var initialPose = getRightSideAutoStartingPose();
        return Commands.sequence(
            // initial settings
            superstructure.setStateCommand(WantedSuperState.IDLE),
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.resetPose(initialPose)),
                new InstantCommand(() -> arm.setWantedState(WantedState.AUTON_START_MOVE_ARM_OUT))),
            new WaitUntilCommand(() -> arm.getPosition() < 0.25),
            new InstantCommand(() -> arm.setWantedState(WantedState.HOLD)),

            // Piece 1

            new ParallelCommandGroup(
                m_factory.trajectoryCmd("StartToF"),
                superstructure.setStateCommand(WantedSuperState.L4)
            ),
            superstructure.setStateCommand(WantedSuperState.IDLE),
            AutoBuilder.pathfindToPose(kAliBLUE2_3[1], K_CONSTRAINTS_Barging),
            superstructure.setStateCommand(WantedSuperState.SCORE),
            new WaitUntilCommand(() -> !bluetooth.hasCoral()),
            superstructure.setStateCommand(WantedSuperState.IDLE),

            // Piece 2

            new ParallelCommandGroup(
                superstructure.setStateCommand(WantedSuperState.TRANSFER_PREP),
                m_factory.trajectoryCmd("FToSource")
            ),
            new ParallelRaceGroup(
                m_factory.trajectoryCmd("SourceToCD"),
                Transfer(superstructure, arm, bluetooth, hopper)
            ),
            superstructure.setStateCommand(WantedSuperState.L4),
            superstructure.setStateCommand(WantedSuperState.IDLE),
            AutoBuilder.pathfindToPose(kAliBLUE4_5[1], K_CONSTRAINTS_Barging),
            superstructure.setStateCommand(WantedSuperState.SCORE),
            new WaitUntilCommand(() -> !bluetooth.hasCoral()),
            superstructure.setStateCommand(WantedSuperState.IDLE),

            //Piece 3

            new ParallelCommandGroup(
                superstructure.setStateCommand(WantedSuperState.TRANSFER_PREP),
                m_factory.trajectoryCmd("DToSource")
            ),
            new ParallelRaceGroup(
                m_factory.trajectoryCmd("SourceToCD"),
                Transfer(superstructure, arm, bluetooth, hopper)
            ),
            superstructure.setStateCommand(WantedSuperState.L4),
            superstructure.setStateCommand(WantedSuperState.IDLE),
            AutoBuilder.pathfindToPose(kAliBLUE4_5[0], K_CONSTRAINTS_Barging),
            superstructure.setStateCommand(WantedSuperState.SCORE),
            new WaitUntilCommand(() -> !bluetooth.hasCoral()),
            superstructure.setStateCommand(WantedSuperState.IDLE)

        );
    }
    private Pose2d getRightSideAutoStartingPose() {
        Alliance alliance = DriverStation.getAlliance().get();
        return alliance == DriverStation.Alliance.Blue ? kBlueRightSideAutoStart : kRedRightSideAutoStart;
    }

    private Command Transfer(Superstructure superstructure, ArmSubsystem arm, BluetoothSubsystem bluetooth, HopperSubsystem hopper) {
        return Commands.sequence(
            superstructure.setStateCommand(WantedSuperState.TRANSFER_PREP),
            new WaitUntilCommand(() -> arm.getPosition() < -0.21),
            superstructure.setStateCommand(WantedSuperState.HANDOFF),
            new WaitUntilCommand(() -> bluetooth.hasCoral()),
            superstructure.setStateCommand(WantedSuperState.SWINGLY_DINGLY),
            bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.IDLE),
            hopper.setWantedStateCommand(HopperSubsystem.WantedState.IDLE)
        );
    }
    
}
