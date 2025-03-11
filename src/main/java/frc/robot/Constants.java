package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.*;

public class Contants {
    public static class ElevatorConstants {
        public static final double kElevatorKp = 5;//5
        public static final double kElevatorKi = 0;
        public static final double kElevatorKd = 0;//
        public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
        public static final double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
        public static final double kElevatorkS = 0.02;
        public static final double kElevatorkG = 0.9;
        public static final double kElevatorkV = 3.8;
        public static final double kElevatorkA = 0.17;
        public static final double kElevatorRampRate = 0.1;
        public static final double kElevatorGearing = 12.0;
        public static final double kElevatorCarriageMass = 4.0;
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kElevatorMinHeightMeters = 0.0;
        public static final double kElevatorMaxHeightMeters = 10.25;
        public static final double kElevatorLength = Inches.of(33).in(Meters);
        public static final Distance kElevatorStartingHeightSim = Meters.of(0.0);
        public static final Angle kElevatorStartingAngle = Degrees.of(-90);
        public static final Distance kLaserCANOffset          = Inches.of(3);
        public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);

        public static double kLowerToScoreHeight =  Units.inchesToMeters(6);
  }

}
