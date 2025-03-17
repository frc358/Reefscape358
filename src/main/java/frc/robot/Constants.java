package frc.robot;

import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static class ElevatorConstants {
    public static final double elevatorGearRatio = 6.0 / 1.0;
    public static final double sprocketDiameter = Units.inchesToMeters(1.75);
  
    public static final int elevatorMainMotorID = 21;
    public static final int elevatorFollowerMotorID = 22;
    public static final int buttonSwitchID = 0;
  
    public static final double maxHeight = Units.inchesToMeters(55);
    public static final double minHeight = 0.0;
  
    public static final double L4Height = Units.inchesToMeters(52); // 28.09
    public static final double L3Height = Units.inchesToMeters(29.5);
    public static final double L2Height = Units.inchesToMeters(13.5);
    public static final double downHeight = Units.inchesToMeters(.75);
    public static final double coralInTheWayAdd = 2.74;
  
    public static final double AlgaeHighHeight = Units.inchesToMeters(8.6 + .3);
    public static final double AlgaeLowHeight = Units.inchesToMeters(1.5);
  
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

    public static final double fastOuttakeSpeed = 0.353;
    public static final double slowOuttakeSpeed = 0.24; // .353 0.2

  }
  
    public static class MiscellaneousConstants {
      public static final double prematchDelay = 2.5;
    }
  }


