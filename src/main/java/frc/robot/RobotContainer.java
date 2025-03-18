// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Outtake;
import frc.robot.Robot;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    @Logged(name = "Elevator")
    private final ElevatorSubsytem elevator = new ElevatorSubsytem();

    @Logged(name = "Outtake")
    private final Outtake outtake = new Outtake();

    public boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //path follower
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

       
        //elevator commands
        NamedCommands.registerCommand(
            "Elevator: L4",
            elevator
                .moveToPosition(ElevatorConstants.L4Height)
                // .onlyIf(outtakeLaserBroken)
                .withTimeout(2.15)
                .asProxy());
        NamedCommands.registerCommand(
            "Elevator: L3",
            elevator
                .moveToPosition(ElevatorConstants.L3Height)
                // .onlyIf(outtakeLaserBroken)
                .withTimeout(4)
                .asProxy());
        NamedCommands.registerCommand(
            "Elevator: L2",
            elevator
                .moveToPosition(ElevatorConstants.L2Height)
                // .onlyIf(outtakeLaserBroken)
                .withTimeout(4)
                .asProxy());
        NamedCommands.registerCommand("score", outtake.autoOuttake().asProxy());
        NamedCommands.registerCommand("stop score", outtake.stopOuttakeMotor());
        


        drivetrain.configureAutoBuilder();
        //configures dashboard to have an autonomose mode chooser
        autoChooser = AutoBuilder.buildAutoChooser("Auto Chooser");
        Shuffleboard.getTab("Auto Chooser").add(autoChooser);
        configureBindings();
        configureElevatorBindings();
        configureOuttakeBindings();
    }

    //gets the chosen auto command from dashboard
    public Command getAutonomousCommand(){
        return autoChooser.getSelected();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);



        
    }

    public void Update_Limelight_Tracking()
    {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast
  
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
        }

    private double leftGoalEstimateX(double distanceFromTagX){
        double neededDistance = 0;
        neededDistance = Constants.VisionConstants.leftGoalX - distanceFromTagX;
        return neededDistance;

    }


    private Command adjustRobotPositionLeft() {
        Update_Limelight_Tracking();
        //check if aptag is detected
        if (m_LimelightHasValidTarget){
            double neededDistance = leftGoalEstimateX(LimelightHelpers.getTX(null));
            //drive neededDistance
            return drivetrain.applyRequest(() ->
            drive.withVelocityX(neededDistance));
        }
        return null;

    }

    private void configureElevatorBindings() {
        
        elevator.setDefaultCommand(elevator.holdPosition());


        //Left half of Coral
        operatorController.leftBumper().onTrue(adjustRobotPositionLeft());
        // Elevator L4
        operatorController
            .b()
            //.and(outtakeLaserBroken)
            .or(operatorController.povLeft().and(operatorController.b()))
            .onTrue(elevator.moveToPosition(ElevatorConstants.L4Height));

         // elevator L3
        operatorController
            .y()
            //.and(outtakeLaserBroken)
            .or(operatorController.povLeft().and(operatorController.y()))
            .onTrue(elevator.moveToPosition(ElevatorConstants.L3Height));

        // elevator L2
        operatorController
            .x()
            //.and(outtakeLaserBroken)
            .or(operatorController.povLeft().and(operatorController.x()))
            .onTrue(elevator.moveToPosition(ElevatorConstants.L2Height));
        // elevator down height
        operatorController.a()
        .onTrue(elevator.moveToPosition(ElevatorConstants.downHeight));

        // home elevator
        operatorController.start().and(operatorController.back()).onTrue(elevator.homeElevator());
        // coral in the way add
        operatorController
            .povRight()
            .onTrue(
                new DeferredCommand(
                    () -> {
                    double newTarget =
                          Units.inchesToMeters(
                              elevator.getPositionInches() + ElevatorConstants.coralInTheWayAdd);
                    return elevator.moveToPosition(newTarget);
                    },
                    Set.of(elevator)));
         // elevator manual down
        operatorController
            .povDown()
            .whileTrue(elevator.downSpeed(0.1))
            .onFalse(elevator.runOnce(() -> elevator.downPosition()));
        // elevator manual up
        operatorController
            .povUp()
            .whileTrue(elevator.upSpeed(0.1))
            .onFalse(elevator.runOnce(() -> elevator.downPosition()));
        //elevator manual up fast
        operatorController
            .povRight()
            .whileTrue(elevator.upSpeed(0.2))
            .onFalse(elevator.runOnce(() -> elevator.downPosition()));
        //elevator manual down fast
        operatorController
            .povLeft()
            .whileTrue(elevator.downSpeed(0.2))
            .onFalse(elevator.runOnce(() -> elevator.downPosition()));

  }
  private void configureOuttakeBindings() {
    // operatorController
    //     .button(OperatorConstants.indexerButton)
    //     .onTrue(outtake.reverseOuttake())
    //     .onFalse(outtake.stopOuttakeMotor());

    //operatorController.start().and(operatorController.back().negate()).onTrue(outtake.fastOuttake()).onFalse(outtake.stopOuttakeMotor());
    operatorController.rightTrigger().onTrue(outtake.fastOuttake()).onFalse(outtake.stopOuttakeMotor());
}

}
