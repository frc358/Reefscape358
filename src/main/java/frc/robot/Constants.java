package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;

public class Constants {
  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 6.0 / 1.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.75);
  
    public static final int elevatorMainMotorID = 21;
    public static final int elevatorFollowerMotorID = 22;
    public static final int buttonSwitchID = 0;
  
    public static final double maxHeight = Units.inchesToMeters(55);
    public static final double minHeight = 0.0;
  
    public static final double autoL4 = Units.inchesToMeters(53.3);
    public static final double autoWiggle = Units.inchesToMeters(53.2999);
    public static final double L4Height = Units.inchesToMeters(53.5);
    public static final double L3Height = Units.inchesToMeters(31.5);
    public static final double L2Height = Units.inchesToMeters(15.5);
    public static final double downHeight = Units.inchesToMeters(.75);
  
  
    public static final double sensorToMechanismRatio =
        elevatorGearRatio / (sprocketDiameter * Math.PI);
  
    public static final double bottomSpeed = 0.1;
  
    public static final LinearVelocity maxVelocity = MetersPerSecond.of(2.26 * 0.95); // 2.26*.9
    public static final LinearAcceleration maxAcceleration =
        maxVelocity.div(Seconds.of(0.5)); // .25
  
    public static final MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity.in(MetersPerSecond))
            .withMotionMagicAcceleration(maxAcceleration.in(MetersPerSecondPerSecond));
  
    public static final Slot0Configs slot0Configs =
        new Slot0Configs()
            .withKS(0.01) // .01
            .withKV(4.75) // 4.14
            .withKA(0.03) // .03
            .withKG(0.37) // .31
            .withKP(27.5)
            .withKI(0.0)
            .withKD(0.13) // 1
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
  
    public static final FeedbackConfigs feedbackConfigs =
        new FeedbackConfigs().withSensorToMechanismRatio(sensorToMechanismRatio);
  
    public static final MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withInverted(
                InvertedValue.CounterClockwise_Positive) // needs to spin left when wires up
            .withNeutralMode(NeutralModeValue.Brake);
    public static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(maxHeight)
            .withForwardSoftLimitEnable(true);
    public static final CurrentLimitsConfigs currentLimitConfigs =
        new CurrentLimitsConfigs().withStatorCurrentLimit(45).withStatorCurrentLimitEnable(true);
  
    public static final TalonFXConfiguration elevatorConfigs =
        new TalonFXConfiguration()
            .withCurrentLimits(currentLimitConfigs)
            .withSlot0(slot0Configs)
            .withMotionMagic(motionMagicConfigs)
            .withFeedback(feedbackConfigs)
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);
  }
  public static class OuttakeConstants {
    public static final int outtakeMotorID = 18;
    public static final int outtakeCurrentLimit = 60;
    public static final int outtakeShutOffLimit = 75;

    public static final double fastOuttakeSpeed = -0.55;
    public static final double slowOuttakeSpeed = -0.25; //
    public static final double slowReverseSpeed = .1;

  }

  public static class VisionConstants {
    public static final double leftGoalX = 24.95;
    public static final double rightGoalX = -14.4;
    public static final double leftGoalY = 1.2;
    public static final double TOLERANCE = 0.01;
  }
  
    public static class MiscellaneousConstants {
      public static final double prematchDelay = 2.5;
    }

    public static class SwerveConstants {

      public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
      public static final LinearVelocity slowModeMaxTranslationalSpeed = FeetPerSecond.of(5);
      public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1.5);

      public static final Time translationZeroToFull = Seconds.of(0.6);
      public static final Time rotationZeroToFull = Seconds.of(0.25);

      public static final LinearAcceleration maxTransationalAcceleration =
          maxTranslationalSpeed.div(translationZeroToFull);
      public static final AngularAcceleration maxAngularAcceleration =
          maxRotationalSpeed.div(rotationZeroToFull);

      public static final double centerToBumber = Units.inchesToMeters(18);
    }

    public static class FieldConstants {
      public static final List<Integer> rejectedTAGS = List.of(4, 14, 15, 5);
      public static final String aprilTagJson = "2025-official-welded";
      public static final Path aprilTagJsonPath =
        Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", aprilTagJson + ".json");

      public static AprilTagFieldLayout aprilTagLayout;

      static {
        try {
          aprilTagLayout = new AprilTagFieldLayout(aprilTagJsonPath);
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      }

      public static final Pose2d redStationLeft =
          new Pose2d(15.826813936233481, 1.0288156509399498, Rotation2d.fromDegrees(125.1));
      public static final Pose2d redStationRight =
          new Pose2d(15.726813936233482, 7.128815650939941, Rotation2d.fromDegrees(-125.1));
      public static final Pose2d blueStationLeft =
          new Pose2d(1.8268139362335205, 7.128815650939941, Rotation2d.fromDegrees(-53.4));
      public static final Pose2d blueStationRight =
          new Pose2d(1.667565608024597, 1.0683921813964927, Rotation2d.fromDegrees(53.4));


      public static final List<Pose2d> redSetupPoses =
          List.of(
              new Pose2d(10.813, 4.019, Rotation2d.fromDegrees(0)), // 180
              new Pose2d(11.934, 2.0774, Rotation2d.fromDegrees(60)), // -120
              new Pose2d(14.176, 2.0774, Rotation2d.fromDegrees(120)), // -60
              new Pose2d(15.297, 4.019, Rotation2d.fromDegrees(180)), // 0
              new Pose2d(14.176, 5.96, Rotation2d.fromDegrees(-120)), // 60
              new Pose2d(11.934, 5.96, Rotation2d.fromDegrees(-60))); // 120

      public static final List<Pose2d> blueSetupPoses =
          List.of(
              new Pose2d(6.725, 4.019, Rotation2d.fromDegrees(180)), // 0
              new Pose2d(5.604, 5.961, Rotation2d.fromDegrees(-120)), // 60
              new Pose2d(3.362, 5.961, Rotation2d.fromDegrees(-60)), // 120
              new Pose2d(2.241, 4.019, Rotation2d.fromDegrees(0)), // 180
              new Pose2d(3.362, 2.078, Rotation2d.fromDegrees(60)), // -120
              new Pose2d(5.604, 2.078, Rotation2d.fromDegrees(120))); // -60


      public static final Pose2d reefBlueAlliance =
          new Pose2d(4.483, 4.019, Rotation2d.fromDegrees(0.0));
      public static final Pose2d reefRedAlliance =
          new Pose2d(13.055, 4.019, Rotation2d.fromDegrees(0));

      public static final Map<Integer, Double> aprilTagAngles = new HashMap<>();

      static {
        aprilTagAngles.put(6, 120.0);
        aprilTagAngles.put(7, 180.0);
        aprilTagAngles.put(8, -120.0);
        aprilTagAngles.put(9, -60.0);
        aprilTagAngles.put(10, 0.0);
        aprilTagAngles.put(11, 60.0);
        aprilTagAngles.put(17, 60.0);
        aprilTagAngles.put(18, 0.0);
        aprilTagAngles.put(19, -60.0);
        aprilTagAngles.put(20, -120.0);
        aprilTagAngles.put(21, 180.0);
        aprilTagAngles.put(22, 120.0);
      }

      public static final Map<Integer, Double> left_aprilTagOffsets = new HashMap<>();

      static {
        left_aprilTagOffsets.put(6, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(7, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(8, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(9, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(10, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(11, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(17, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(18, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(19, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(20, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(21, Units.inchesToMeters(6.488));
        left_aprilTagOffsets.put(22, Units.inchesToMeters(6.488));
      }

      public static final Map<Integer, Double> right_aprilTagOffsets = new HashMap<>();

      static {
        right_aprilTagOffsets.put(6, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(7, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(8, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(9, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(10, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(11, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(17, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(18, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(19, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(20, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(21, Units.inchesToMeters(-6.488));
        right_aprilTagOffsets.put(22, Units.inchesToMeters(-6.488));
      }

      public static class ReefDefinitePoses {
        public static final List<Pose2d> blueReefDefiniteLeftPoses =
            List.of(
                new Pose2d(5.803, 3.858, Rotation2d.fromDegrees(180)),
                new Pose2d(4.999, 2.806, Rotation2d.fromDegrees(120)),
                new Pose2d(3.691, 2.975, Rotation2d.fromDegrees(60)),
                new Pose2d(3.181, 4.187, Rotation2d.fromDegrees(0)),
                new Pose2d(3.975, 5.244, Rotation2d.fromDegrees(-60)),
                new Pose2d(5.279, 5.077, Rotation2d.fromDegrees(-120)));

        public static final List<Pose2d> blueReefDefiniteRightPoses =
            List.of(
                new Pose2d(5.803, 4.187, Rotation2d.fromDegrees(180)),
                new Pose2d(5.288, 2.970, Rotation2d.fromDegrees(120)),
                new Pose2d(3.976, 2.807, Rotation2d.fromDegrees(60)),
                new Pose2d(3.186, 3.859, Rotation2d.fromDegrees(0)),
                new Pose2d(3.690, 5.077, Rotation2d.fromDegrees(-60)),
                new Pose2d(4.993, 5.245, Rotation2d.fromDegrees(-120)));

        public static final List<Pose2d> redReefDefiniteRightPoses =
            List.of(
                new Pose2d(14.372, 4.189, Rotation2d.fromDegrees(180)),
                new Pose2d(13.858, 2.976, Rotation2d.fromDegrees(120)),
                new Pose2d(12.553, 2.808, Rotation2d.fromDegrees(60)),
                new Pose2d(11.755, 3.849, Rotation2d.fromDegrees(0)),
                new Pose2d(12.267, 5.078, Rotation2d.fromDegrees(-60)),
                new Pose2d(13.572, 5.238, Rotation2d.fromDegrees(-120)));

        public static final List<Pose2d> redReefDefiniteLeftPoses =
            List.of(
                new Pose2d(14.373, 3.860, Rotation2d.fromDegrees(180)),
                new Pose2d(13.578, 2.814, Rotation2d.fromDegrees(120)),
                new Pose2d(12.269, 2.980, Rotation2d.fromDegrees(60)),
                new Pose2d(11.755, 4.189, Rotation2d.fromDegrees(0)),
                new Pose2d(12.553, 5.243, Rotation2d.fromDegrees(-60)),
                new Pose2d(13.854, 5.076, Rotation2d.fromDegrees(-120)));
      }
    }

    public static class AutoConstants {

       public static final PathConstraints slowPathConstraints =
        new PathConstraints(.1, 1, Units.degreesToRadians(180), Units.degreesToRadians(360));
    }
  }


