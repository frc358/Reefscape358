// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.TimedRobot;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.Outtake;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.TurnToReef;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    //set robot centric alignment for aligning to coral
    final SwerveRequest.RobotCentric align = new SwerveRequest.RobotCentric();
    
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    @Logged(name = "Elevator")
    private final ElevatorSubsytem elevator = new ElevatorSubsytem();

    @Logged(name = "Outtake")
    private final Outtake outtake = new Outtake();

    public boolean m_LimelightHasValidTarget = false;
    public RawFiducial[] fiducials;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SlewRateLimiter slewX = new SlewRateLimiter(TunerConstants.DRIVE_SLEW_RATE);
    private final SlewRateLimiter slewY = new SlewRateLimiter(TunerConstants.DRIVE_SLEW_RATE);
    private final SlewRateLimiter slewTheta = new SlewRateLimiter(TunerConstants.DRIVE_SLEW_RATE * 2);

    //path follower
    private final SendableChooser<Command> autoChooser;

    public static LimelightHelpers m_limelight;
    

    public RobotContainer() {
        
       
        //elevator commands
        NamedCommands.registerCommand(
            "Elevator: L4",
            elevator
                .moveToPosition(ElevatorConstants.L4Height)
                // .onlyIf(outtakeLaserBroken)
                .withTimeout(3)
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
        NamedCommands.registerCommand("Elevator: Down", elevator.moveToPosition(ElevatorConstants.minHeight).withTimeout(2).andThen(outtake.stopOuttake().withTimeout(1)));
        NamedCommands.registerCommand("Elevator: down", elevator.moveToPosition(ElevatorConstants.minHeight).withTimeout(2).andThen(outtake.stopOuttake().withTimeout(1).asProxy()));

        //Outtake Commands
        NamedCommands.registerCommand("score", outtake.fastOuttake().withTimeout(2.0).asProxy());
        NamedCommands.registerCommand("stop score", outtake.stopOuttakeMotor().asProxy());
        NamedCommands.registerCommand("HP intake", outtake.slowOuttake().withTimeout(1.5));
        //Sequenced Commands
        NamedCommands.registerCommand("L4 Then Shoot", elevator.moveToPosition(ElevatorConstants.autoL4).withTimeout(1.5).andThen(outtake.slowOuttake().withTimeout(2.5)));
        NamedCommands.registerCommand("L2 Then Shoot", elevator.moveToPosition(ElevatorConstants.L2Height).withTimeout(1).andThen(outtake.slowOuttake().withTimeout(2.5)));
    
        //Alignment Commands
        NamedCommands.registerCommand("Path Find To Setup", drivetrain.pathFindToSetup());
        NamedCommands.registerCommand("Turn To Reef", new TurnToReef(drivetrain).withTimeout(2));

        drivetrain.configureAutoBuilder();
        //configures dashboard to have an autonomose mode chooser
        autoChooser = AutoBuilder.buildAutoChooser("Blue Middle");
        Shuffleboard.getTab("Auto Chooser").add(autoChooser);

        //configure driver and operator xbox bindings
        configureBindings();
        configureElevatorBindings();
        configureOuttakeBindings();
        configureAlignmentBindings();
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
                drive.withVelocityX(slewX.calculate(Math.pow(-joystick.getLeftY(), 3)) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(slewY.calculate(Math.pow(-joystick.getLeftX(), 3)) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate((Math.pow(-joystick.getRightX(), 3)) * MaxAngularRate)) // Drive counterclockwise with negative X (left)

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

    private void configureElevatorBindings() {
        
        elevator.setDefaultCommand(elevator.holdPosition());

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
        .onTrue(elevator.moveToPosition(ElevatorConstants.minHeight));

        // home elevator
        operatorController.start().and(operatorController.back()).onTrue(elevator.homeElevator());
         // elevator manual down
        operatorController
            .povDown()
            .whileTrue(elevator.downSpeed(0.1))
            .onFalse(elevator.runOnce(() -> elevator.downPosition()));
        //elevator manual up fast
        operatorController
            .povRight()
            .whileTrue(elevator.upSpeed(0.6))
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
        operatorController.leftTrigger().onTrue(outtake.slowOuttake()).onFalse(outtake.stopOuttakeMotor());
        operatorController.leftStick().onTrue(outtake.reverseOuttake()).onFalse(outtake.stopOuttakeMotor());
    }
    
    private void configureAlignmentBindings(){
        //operatorController.leftBumper().onTrue(drivetrain.pathFindToSetup());
        joystick.rightBumper().onTrue(new TurnToReef(drivetrain));    

        //drive forward robot centric
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(.1)));        //drive left/right
        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(-.125).withRotationalRate(0)));
        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0).withVelocityY(.125).withRotationalRate(0)));
        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(-.1).withVelocityY(0).withRotationalRate(0)));

        //operatorController.leftBumper().onTrue(drivetrain.pathFindToSetup().andThen(new TurnToReef(drivetrain).andThen(drivetrain.reefAlign(true))));
        //operatorController.rightBumper().onTrue(drivetrain.pathFindToSetup().andThen(new TurnToReef(drivetrain).andThen(drivetrain.reefAlign(false))));        
    }


double limelight_calc_distance(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 15; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 13.5; 

    // distance from the target to the floor
    double goalHeightInches = 7; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
}


 // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= MaxAngularRate;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= MaxSpeed; //max speed is 2.5
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -slewX.calculate(MathUtil.applyDeadband(joystick.getLeftY(), 0.02))
            * MaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -slewY.calculate(MathUtil.applyDeadband(joystick.getLeftX(), 0.02))
            * MaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -slewTheta.calculate(MathUtil.applyDeadband(joystick.getRightX(), 0.02))
            * MaxAngularRate;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    /* 
    if(true)
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }
        */
  }
}
