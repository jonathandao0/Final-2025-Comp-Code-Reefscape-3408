// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawWheelSubsystem;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.commands.AlgaeShoot;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.PID_SetClawPosition;
import frc.robot.commands.PID_SetLiftPosition;
import frc.robot.commands.CoralShoot;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.LiftConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Optional<Alliance> ally = DriverStation.getAlliance();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //my additions
    final         CommandXboxController driverXbox = new CommandXboxController(0);
    final         CommandXboxController mechXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...

    private final LiftSubsystem m_lift = new LiftSubsystem();
    private final ClawSubsystem m_claw = new ClawSubsystem();
    private final ClawWheelSubsystem m_clawWheel = new ClawWheelSubsystem();
    private final HangSubsystem m_hang = new HangSubsystem();

    UsbCamera coralCamera;

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("N_ClawPos", new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2, true));
        NamedCommands.registerCommand("L4_LiftPos", new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint4+2,true));
        NamedCommands.registerCommand("CoralShoot", new CoralShoot(m_claw));
        NamedCommands.registerCommand("L1_LiftPos", new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint1, true));
        

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("Auto Paths");
        // Send the driveTrain to the dashboard to help us know what command the Subsystem is running
        SmartDashboard.putData("Swerve", drivetrain);
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-driverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(-driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        driverXbox.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverXbox.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverXbox.back().and(driverXbox.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverXbox.back().and(driverXbox.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverXbox.start().and(driverXbox.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverXbox.start().and(driverXbox.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverXbox.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        //REEFSCAPE SPECIFIC BINDINGS
        mechXbox.leftTrigger().whileTrue(m_clawWheel.wheelForward());
        mechXbox.leftTrigger().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));
        //mechXbox.leftTrigger().whileTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));
        mechXbox.leftBumper().whileTrue(m_clawWheel.wheelBackward());
        mechXbox.leftBumper().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));
        mechXbox.rightTrigger().whileTrue(new CoralIntake(m_claw, m_clawWheel));
        mechXbox.rightBumper().whileTrue(m_clawWheel.wheelBackward());
        
        mechXbox.a().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint1, false));
        mechXbox.a().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2,false));
        mechXbox.x().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint2, false));
        mechXbox.x().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2,false));
        mechXbox.y().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint3, false));
        mechXbox.y().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2,false));
        mechXbox.b().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kLiftSetpoint4, false));
        mechXbox.b().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint5,false));
        
        mechXbox.back().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kAlgaeSetpoint1, false));
        mechXbox.back().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));
        mechXbox.start().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kAlgaeSetpoint2, false));
        mechXbox.start().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));
        mechXbox.rightStick().whileTrue(new PID_SetLiftPosition(m_lift, LiftConstants.kAlgaeSetpoint3, false));
        mechXbox.rightStick().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));

        mechXbox.povUp().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint1,false));
        mechXbox.povRight().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint2,false));
        mechXbox.povDown().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint3,false));
        mechXbox.povLeft().onTrue(new PID_SetClawPosition(m_claw, ClawConstants.kClawSetpoint5,false));

        mechXbox.leftStick().whileTrue(new AlgaeShoot(m_claw, m_clawWheel, ClawConstants.kClawSetpoint2, m_lift));


        driverXbox.a().whileTrue(generateDriveToTargetCommand());

        //driverXbox.y().whileTrue(new Hang(m_hang));

        //driverXbox.a().whileTrue(CommandSwerveDrivetrain.driveToPose(new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0))) );
        //driverXbox.b().onTrue(CommandSwerveDrivetrain.driveToPose(new Pose2d(new Translation2d(2, 2), Rotation2d.fromDegrees(0))) );
        

        //driverXbox.leftTrigger().whileTrue(m_lift.lowerLift());
        //driverXbox.rightTrigger().whileTrue(m_lift.raiseLift());
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Move the path logic to a function
    public Command generateDriveToTargetCommand() {
        /** You must 'defer' the command in order for it to work properly. Commands are 'static' in that they
         * initialize everything on code startup. In this case, the logic below will always use the pose kZero, because
         * the code is processed on boot. To avoid this, we use 'Commands.defer()' to have the command be generated when
         * we call it from the button press in order to process the logic properly.
         *
         * For a historical perspective, Command-based programming does this in order to 're-use' commands to avoid the
         * memory/CPU overhead costs that would be incurred if commands were constantly generated, which can have an
         * impact on the code's performance due to the roboRIO's specs. So long as you don't do this too much, using
         * 'Commands.defer()' shouldn't cause issues.
         *
         */
        return Commands.defer(()-> {
            var allianceColor = DriverStation.getAlliance();
            Pose2d targetPose = Pose2d.kZero;

            if (allianceColor.isPresent()) {
                //blue alliance targets
                if (allianceColor.get() == Alliance.Blue) {
                    targetPose = new Pose2d(3.9, 5.4, Rotation2d.fromDegrees(-88.9));
                } else {
                    targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
                }
            }

            // AutoBuilder.pathfindToPose() returns a command, which we use to follow the path
            return AutoBuilder.pathfindToPose(
                    targetPose,
                    constraints,
                    0.0 // Goal end velocity in meters/sec
            );
        },
        // Use 'Set.of()' to set the Command's Subsystem requirements for a deferred command
        Set.of(drivetrain)).withName("DriveToPose"); // Name the command to help with debugging
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
