//ojit
// 24 intake
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Joystick joy1 = new Joystick(0);
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final CANSparkMax FR = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax BR = new CANSparkMax(12, MotorType.kBrushless);

  private final MotorControllerGroup rightSide = new MotorControllerGroup(FR, BR);
  public RelativeEncoder FR_encoder = FR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  public RelativeEncoder BR_encoder = BR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  private final CANSparkMax FL = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax BL = new CANSparkMax(22, MotorType.kBrushless);
  private final MotorControllerGroup leftSide = new MotorControllerGroup(FL, BL);
  public RelativeEncoder FL_encoder = FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  public RelativeEncoder BL_encoder = BL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftSide, rightSide);

  PIDController pidlimelight = new PIDController(0.005, 0, 0);
  PIDController controller = new PIDController(0.6, 0, 0.00000);
  PIDController controllerang = new PIDController(0.016, 0, 0.009);
  PIDController controllerangle = new PIDController(0.0009310793, 0, 0.00);
  double flag = 1;
  double setpoint = 0;
  final double kP = 0.0145;
  final double kPang = 0.0026;
  final double kI = 0.005;
  final double kD = 0.0009;
  double errorsum = 0;
  double lasttimestamp = 0;
  final double ilimit = 50;
  double lasterror = 0;
  double x[] = { -1, 0, 0 };

  double y[] = { 0, 0, 0, 0 };
  double dx = 0.1;
  double dy = 1;
  double angleinuse = 0;
  double distance = 0;
  double d_theta = 0;
  double theta_dir = 0;
  int i = 0;
  Pose2d pose;
  int f = 1;
  double f1 = 0;
  double f2 = 0;
  double f3 = 0;
  double f4 = 0;
  double f5 = 0;
  double f6 = 0;
  double f7 = 0;
  double f8 = 0;
  double f9 = 0;
  double ty;
  double angletotake;
  double Trans_Lmot = 0;
  double Trans_Rmot = 0;
  double angle;
  double a, b = 0;
  private DifferentialDriveOdometry m_odometry;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   * 
   */
  @Override
  public void robotInit() {
    FR.setInverted(true);
    BR.setInverted(true);
    FL.setInverted(false);
    BL.setInverted(false);
    FR_encoder.setPosition(0.0);
    BR_encoder.setPosition(0.0);
    FL_encoder.setPosition(0.0);
    BL_encoder.setPosition(0.0);
    gyro.zeroYaw();
    gyro.reset();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-gyro.getAngle()));

    m_robotContainer = new RobotContainer();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    pose = m_odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
        (FL_encoder.getPosition() * Math.PI * Units.inchesToMeters(6)) / 7.31,
        (FR_encoder.getPosition() * Math.PI * Units.inchesToMeters(6)) / 7.31);
    SmartDashboard.putNumber("angle POSE", pose.getRotation().getDegrees());

    // double angle = gyro.getAngle() % 360;
    // SmartDashboard.putNumber("ang", angle);
    SmartDashboard.putNumber("X", pose.getX());
    SmartDashboard.putNumber("Y", pose.getY());
  }

  /**
   * This func.---------t.ion is called once each time the robot enters Disabled
   * mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // leftturn();
  }

  @Override
  public void teleopInit() {
    // left.setSelectedSensorPosition(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    FR_encoder.setPosition(0.0);
    BR_encoder.setPosition(0.0);
    FL_encoder.setPosition(0.0);
    BL_encoder.setPosition(0.0);
    gyro.zeroYaw();
    // gyro.setAngleAdjustment(180);
    stop();
    // left.setA();
    // lasttimestamp = Timer.getFPGATimestamp();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  public double[] speedcontrol(int i) {

    if (Math.abs(x[i] - a) > 0.2 || Math.abs(y[i] - b) > 0.2) {
      Trans_Lmot = controller.calculate(0, distance) - controllerang.calculate(0, d_theta * theta_dir);
      Trans_Rmot = controller.calculate(0, distance) + controllerang.calculate(0, d_theta * theta_dir);
    } else {
      Trans_Lmot = 0;
      Trans_Rmot = 0;
    }
    if (Math.abs(Trans_Lmot) > 0.2 || Math.abs(Trans_Rmot) > 0.2) {
      if (Math.abs(Trans_Lmot) > Math.abs(Trans_Rmot)) {
        Trans_Rmot = 0.2 * Trans_Rmot / Math.abs(Trans_Lmot);
        Trans_Lmot = 0.2 * Trans_Lmot / Math.abs(Trans_Lmot);
        // MathUtil.F(Trans_Lmot, -0.2, 0.2);
      } else {
        Trans_Lmot = 0.2 * Trans_Lmot / Math.abs(Trans_Rmot);
        Trans_Rmot = 0.2 * Trans_Rmot / Math.abs(Trans_Rmot);
        // MathUtil.clamp(Trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { Trans_Lmot, Trans_Rmot };
    return speed;
  }

  public double[] speedcontrolAngular(int i) {

    // if (Math.abs(x[i] - a) > 0.2 || Math.abs(y[i] - b) > 0.2) {
    // Trans_Lmot = controller.calculate(0, distance) - controllerang.calculate(0,
    // d_theta * theta_dir);
    // Trans_Rmot = controller.calculate(0, distance) + controllerang.calculate(0,
    // d_theta * theta_dir);
    // } else {
    // Trans_Lmot = 0;
    // Trans_Rmot = 0;
    // }

    Trans_Lmot = -controllerang.calculate(0, d_theta * theta_dir);
    Trans_Rmot = controllerang.calculate(0, d_theta * theta_dir);
    if (Math.abs(Trans_Lmot) > 0.2 || Math.abs(Trans_Rmot) > 0.2) {
      if (Math.abs(Trans_Lmot) > Math.abs(Trans_Rmot)) {
        Trans_Rmot = 0.2 * Trans_Rmot / Math.abs(Trans_Lmot);
        Trans_Lmot = 0.2 * Trans_Lmot / Math.abs(Trans_Lmot);
        // MathUtil.F(Trans_Lmot, -0.2, 0.2);
      } else {
        Trans_Lmot = 0.2 * Trans_Lmot / Math.abs(Trans_Rmot);
        Trans_Rmot = 0.2 * Trans_Rmot / Math.abs(Trans_Rmot);
        // MathUtil.clamp(Trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { Trans_Lmot, Trans_Rmot };
    return speed;
  }

  public void stop() {
    FR.set(0.0);
    BR.set(0.0);
    FL.set(0.0);
    BL.set(0.0);
  }

  public void resetall() {
    gyro.zeroYaw();
    FR_encoder.setPosition(0.0);
    BR_encoder.setPosition(0.0);
    FL_encoder.setPosition(0.0);
    BL_encoder.setPosition(0.0);
  }

  public void resetdistance() {
    FR_encoder.setPosition(0.0);
    BR_encoder.setPosition(0.0);
    FL_encoder.setPosition(0.0);
    BL_encoder.setPosition(0.0);
  }

  public void resetpid() {
    controller.reset();
    controllerang.reset();
    controllerangle.reset();
  }

  public void resetangle() {
    gyro.zeroYaw();
  }

  public void armfuction() {

  }

  public void stepwise(int i) {

    if (f == 1) {
      getDesiredstate(i, 0, 180);
      if (Math.abs(d_theta) > 1) {
        FL.set(speedcontrolAngular(i)[0]);
        BL.set(speedcontrolAngular(i)[0]);
        FR.set(speedcontrolAngular(i)[1]);
        BR.set(speedcontrolAngular(i)[1]);
      } else {
        stop();
        // controller.reset();
        // controllerang.reset();
        // controllerangle.reset();

        f = 2;
      }
    }
    if (f == 2) {
      getDesiredstate(i, 0, 180);
      if (Math.abs(distance) > 0.5) {
        FL.set(speedcontrol(i)[0]);
        BL.set(speedcontrol(i)[0]);
        FR.set(speedcontrol(i)[1]);
        BR.set(speedcontrol(i)[1]);
      } else {
        stop();
        // resetpid();
        // resetdistance();
        // controller.reset();
        // controllerang.reset();

        f = 4;

      }
    }
    if (f == 3) {
      getDesiredstate(i, 1, 180);
      if (Math.abs(d_theta) > 5) {
        FL.set(speedcontrolAngular(i)[0]);
        BL.set(speedcontrolAngular(i)[0]);
        FR.set(speedcontrolAngular(i)[1]);
        BR.set(speedcontrolAngular(i)[1]);
      } else {
        stop();
        // controller.reset();
        // controllerang.reset();
        // controllerangle.reset();
        // resetdistance();
        f = 0;
      }
    }
    if (f == 4) {
      getDesiredstate(i, 2, 180);
      if (Math.abs(x[i] - a) > 0.1 || Math.abs(y[i] - b) > 0.1) {
        FL.set(speedcontrol(i)[0]);
        BL.set(speedcontrol(i)[0]);
        FR.set(speedcontrol(i)[1]);
        BR.set(speedcontrol(i)[1]);
      } else {
        stop();
        // controller.reset();
        // controllerang.reset();
        // controllerangle.reset();
        // resetdistance();
        f = 0;
      }
    }
    if (f == 0) {

      stop();
      f2 = 0;
    }
  }

  double ps1 = 0;
  double ps2 = 0;

  public void getRobotstate() {
    a = pose.getX();
    b = pose.getY();
    angle = -gyro.getAngle() % 360;
  }

  /*
   * motionType:
   * Coordinate Maintain = 0;
   * Angular = 1;
   * Linear = 2;
   */
  public void getDesiredstate(int i, int motionType, double desiredAngle) {

    if (motionType == 0) {
      distance = Math.sqrt(Math.pow((a - x[i]), 2) + Math.pow((b - y[i]), 2));
      angletotake = Math.toDegrees(Math.atan2((y[i] - b), (x[i] - a)));
    } else if (motionType == 2) {
      distance = Math.sqrt(Math.pow((a - x[i]), 2) + Math.pow((b - y[i]), 2)) * (x[i] - a) * x[i]
          / (Math.abs(x[i] - a) * Math.abs(x[i]));
      // angletotake = Math.toDegrees(Math.atan2((y[i] - b), (x[i] - a)));
      angletotake = desiredAngle;
    } else {
      distance = Math.sqrt(Math.pow((a - x[i]), 2) + Math.pow((b - y[i]), 2));
      angletotake = desiredAngle;
    }
    angletotake = (angletotake + 720) % 360;
    d_theta = angletotake - angle;
    theta_dir = d_theta / Math.abs(d_theta);
    if (Math.abs(d_theta) >= 180) {
      d_theta = 360 - Math.abs(d_theta);
      theta_dir *= -1;
    } else {
      d_theta = Math.abs(d_theta);
      theta_dir *= 1;
    }
    d_theta = d_theta * theta_dir;
  }

  public void displayCells() {
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("x", a);
    SmartDashboard.putNumber("y", b);
    SmartDashboard.putNumber("angle to take", angletotake);
    SmartDashboard.putNumber("angle to take with direction", d_theta * theta_dir);
    SmartDashboard.putNumber("D Theta", d_theta);
    SmartDashboard.putNumber("Dir", theta_dir);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("f", f);
    SmartDashboard.putNumber("Trans_Lmot", Trans_Lmot);
    SmartDashboard.putNumber("Trans_Rmot", Trans_Rmot);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (joy1.getRawButton(1)) {
      getRobotstate();
      stepwise(0);
      displayCells();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
