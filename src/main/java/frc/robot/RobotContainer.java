// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ClimberConstants.*;
import frc.robot.Constants.ElevatorConstants.*;
import frc.robot.Constants.HopperConstants.*;
import frc.robot.Superstructure.WantedSuperState;
import frc.robot.commands.TransferCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.AlgaeGripper.AlgaeGripperIOTalonFX;
import frc.robot.subsystems.AlgaeGripper.AlgaeGripperSubsystem;
import frc.robot.subsystems.Arm.ArmIOSparkMax;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Arm.ArmSubsystem.WantedState;
import frc.robot.subsystems.Bluetooth.BluetoothIOSparkMax;
import frc.robot.subsystems.Bluetooth.BluetoothSubsystem;
import frc.robot.subsystems.Climber.ClimberIOSparkMax;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Hopper.HopperIO;
import frc.robot.subsystems.Hopper.HopperIOSparkMax;
import frc.robot.subsystems.Hopper.HopperSubsystem;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ReefPoses.*;

import java.util.Set;
import java.util.function.Supplier;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  HopperSubsystem hopper;
  Superstructure superstructure;
  ElevatorSubsystem elevator;
  ClimberSubsystem climber;
  ArmSubsystem arm;
  BluetoothSubsystem bluetooth;
  AlgaeGripperSubsystem algae;
  CommandSwerveDrivetrain drivetrain;
  LEDs leds;
  AutoFactory factory;
  public final AutoRoutines autos; 
  private final SendableChooser<Command> autoChooser;
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = 3 * Math.PI;
    private double gyro = 1;
    // Swerve drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed*0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
  private final CommandXboxController operator = 
      new CommandXboxController(1);
  int level = 3;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    hopper = new HopperSubsystem(new HopperIOSparkMax(6,7));
    elevator = new ElevatorSubsystem(new ElevatorIOSparkMax(1, 2));
    climber = new ClimberSubsystem(new ClimberIOSparkMax(8, 9));
    arm = new ArmSubsystem(new ArmIOSparkMax(3));
    bluetooth = new BluetoothSubsystem(new BluetoothIOSparkMax(5));
    algae = new AlgaeGripperSubsystem(new AlgaeGripperIOTalonFX(21));
    drivetrain = TunerConstants.createDrivetrain();
    leds = new LEDs(new AddressableLED(9), new AddressableLEDBuffer(138), bluetooth, algae);
    superstructure = new Superstructure(hopper, elevator, climber, bluetooth, arm);
    factory = drivetrain.createAutoFactory();
    autos = new AutoRoutines(factory, superstructure, drivetrain, arm, elevator, bluetooth, hopper);
    autoChooser = new SendableChooser<>();

    autoChooser.addOption("Drive Test", autos.DriveTest());
    autoChooser.addOption("Branches FCD", autos.BranchesFCD());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(-m_driverController.getLeftY() * MaxSpeed *gyro)
                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * gyro)
                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
            )
    );
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * m_driverControllers}.
   */
  private void configureBindings() {
    

    m_driverController.leftBumper().onTrue(
      superstructure.setStateCommand(Superstructure.WantedSuperState.L2)
    ).onFalse(
      superstructure.setStateCommand(WantedSuperState.IDLE)
    );
    m_driverController.rightBumper().onTrue(
      superstructure.setStateCommand(Superstructure.WantedSuperState.L3)
    ).onFalse(
      superstructure.setStateCommand(WantedSuperState.IDLE)
    );
    m_driverController.y().onTrue(
      superstructure.setStateCommand(WantedSuperState.SCORE)
    )
    .onFalse(
      superstructure.setStateCommand(WantedSuperState.IDLE)
    );
    operator.leftTrigger().whileTrue(
      new SequentialCommandGroup(
        superstructure.setStateCommand(WantedSuperState.TRANSFER_PREP),
        new WaitUntilCommand(() -> arm.getPosition() < -0.21),
        superstructure.setStateCommand(WantedSuperState.HANDOFF),
        new WaitUntilCommand(() -> bluetooth.hasCoral()),
        superstructure.setStateCommand(WantedSuperState.SWINGLY_DINGLY),
        bluetooth.setWantedStateCommand(BluetoothSubsystem.WantedState.IDLE),
        hopper.setWantedStateCommand(HopperSubsystem.WantedState.IDLE)
      )
    )
    .onFalse(superstructure.setStateCommand(WantedSuperState.IDLE));

    m_driverController.b().whileTrue(
            new SequentialCommandGroup(
            new ConditionalCommand(
              new ConditionalCommand(
                superstructure.setStateCommand(Superstructure.WantedSuperState.L3), 
                superstructure.setStateCommand(Superstructure.WantedSuperState.L4), () -> level == 3)
              , 
              superstructure.setStateCommand(Superstructure.WantedSuperState.L2), () -> level != 2),
              new WaitCommand(0.5),
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(drivetrain.getNearestReefPoseRight(), K_CONSTRAINTS_Barging),
                Set.of() // required subsystem dependencies if any
            )
            )
        );
        m_driverController.x().whileTrue(
            new SequentialCommandGroup(
            new ConditionalCommand(
              new ConditionalCommand(
                superstructure.setStateCommand(Superstructure.WantedSuperState.L3), 
                superstructure.setStateCommand(Superstructure.WantedSuperState.L4), () -> level == 3)
              , 
              superstructure.setStateCommand(Superstructure.WantedSuperState.L2), () -> level != 2),
            new WaitCommand(0.5),
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(drivetrain.getNearestReefPoseLeft(), K_CONSTRAINTS_Barging),
                Set.of() // required subsystem dependencies if any
            )
            )
        );
        m_driverController.povLeft().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        m_driverController.povRight().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(-0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        m_driverController.povUp().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        m_driverController.povDown().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(-0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        operator.povUp().onTrue(new InstantCommand(() -> level++));
        operator.povDown().onTrue(new InstantCommand(() -> level--));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Commands.none();
  }
  public void setRobotStateIdle() {
    superstructure.setWantedSuperState(WantedSuperState.IDLE);
  }

}
