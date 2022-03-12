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
  private final WPI_TalonSRX left = new WPI_TalonSRX(6);
  // CANCoder coder = new CANCoder(4);

  private final CANSparkMax FR = new CANSparkMax(11, MotorType.kBrushless);
  private final CANSparkMax BR = new CANSparkMax(12, MotorType.kBrushless);
  private final CANSparkMax shooter = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorControllerGroup rightSide = new MotorControllerGroup(FR, BR);
  public RelativeEncoder FR_encoder = FR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  public RelativeEncoder BR_encoder = BR.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  private final CANSparkMax FL = new CANSparkMax(21, MotorType.kBrushless);
  private final CANSparkMax BL = new CANSparkMax(22, MotorType.kBrushless);
  private final MotorControllerGroup leftSide = new MotorControllerGroup(FL, BL);
  public RelativeEncoder FL_encoder = FL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
  public RelativeEncoder BL_encoder = BL.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftSide, rightSide);

  // private final DifferentialDriveOdometry m_odometry = new
  // DifferentialDriveOdometry(getHeading());

  // ProfiledPIDController controller = new ProfiledPIDController(
  // 0.07, 0, 0,
  // new TrapezoidProfile.Constraints(5, 1));

  // ProfiledPIDController controllerang = new ProfiledPIDController(
  // 0.0099, 0.00, 0.00001,
  // new TrapezoidProfile.Constraints(5, 1));

  // ProfiledPIDController controllerangle = new ProfiledPIDController(
  // 0.0156, 0, 0.001,
  // new TrapezoidProfile.Constraints(5, 1), 0.02);
  // ProfiledPIDController controller1 = new ProfiledPIDController(
  // 0.4, 0., 0.001,
  // new TrapezoidProfile.Constraints(5, 1));

  PIDController pidlimelight = new PIDController(0.005, 0, 0);
  PIDController controller = new PIDController(0.6, 0, 0.00000);
  PIDController controllerang = new PIDController(0.016, 0, 0.009);
  PIDController controllerangle = new PIDController(0.0009310793, 0, 0.00);

  // private WPI_TalonSRX m_leftMotor = new WPI_TalonSRX(6);

  // private WPI_TalonSRX m_rightMotor=new WPI_TalonSRX(7);
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
  double x[] = { 2, 0, 0 };

  double y[] = { 0, 0, 0, 0 };
  double dx = 0.1;
  double dy = 1;
  double angleinuse = 0;
  double distanceinuse = 0;
  int i = 0;
  Pose2d pose;
  double f = 1;
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
  private DifferentialDriveOdometry m_odometry;
  // public Rotation2d getHeading() {
  // return Rotation2d.fromDegrees(-gyro.getAngle() % 360);
  // }

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

  public double speedcontrol(double speed) {
    // if(speed>0.1)
    // {
    // return 0.1;
    // }
    // if(speed<-0.1)
    // {
    // return -0.1;
    // }
    // return speed;

    return MathUtil.clamp(speed, -0.2, 0.2);
  }

  public double[] speedcontrol2(int i) {
    a = pose.getX();
    b = pose.getY();

    double totalDistance = (FL_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6)) / 10.71)
        + ((BR_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6))) / 10.71);
    double distance = totalDistance / 2;
    double angle = -gyro.getAngle() % 360;
    angletotake = Math.toDegrees(Math.atan2((y[i] - b), (x[i] - a)));
    // angletotake = Math.toDegrees(Math.atan2(1, 1));
    // if(an)

    angletotake = (angletotake + 720) % 360;
    double d = Math.sqrt(Math.pow((a - x[i]), 2) + Math.pow((b - y[i]), 2));
    double d_theta = angletotake - angle;
    double theta_dir = d_theta / Math.abs(d_theta);

    if (Math.abs(d_theta) >= 180) {
      d_theta = 360 - Math.abs(d_theta);
      theta_dir *= -1;
    } else {
      d_theta = Math.abs(d_theta);
      theta_dir *= 1;
    }
    SmartDashboard.putNumber("d", d);
    SmartDashboard.putNumber("x", a);
    SmartDashboard.putNumber("y", b);
    SmartDashboard.putNumber("angle to take", angletotake);
    SmartDashboard.putNumber("angle to take with direction", d_theta * theta_dir);
    SmartDashboard.putNumber("D Theta", d_theta);
    SmartDashboard.putNumber("Dir", theta_dir);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("speed", controllerang.calculate(0, d_theta * theta_dir));
    if (Math.abs(x[i] - a) > 0.2 || Math.abs(y[i] - b) > 0.2) {
      Trans_Lmot = controller.calculate(0, d) - controllerang.calculate(0, d_theta * theta_dir);
      Trans_Rmot = controller.calculate(0, d) + controllerang.calculate(0, d_theta * theta_dir);
    } else {
      Trans_Lmot = 0;
      Trans_Rmot = 0;
    }
    if (Math.abs(Trans_Lmot) > 0.1 || Math.abs(Trans_Rmot) > 0.1) {
      if (Math.abs(Trans_Lmot) > Math.abs(Trans_Rmot)) {
        Trans_Rmot = 0.1 * Trans_Rmot / Math.abs(Trans_Lmot);
        Trans_Lmot = 0.1 * Trans_Lmot / Math.abs(Trans_Lmot);
        // MathUtil.F(Trans_Lmot, -0.2, 0.2);
      } else {
        Trans_Lmot = 0.1 * Trans_Lmot / Math.abs(Trans_Rmot);
        Trans_Rmot = 0.1 * Trans_Rmot / Math.abs(Trans_Rmot);
        // MathUtil.clamp(Trans_Rmot, -0.2, 0.2);
      }

    }
    double speed[] = { Trans_Lmot, Trans_Rmot };
    // double speed[] = { 0.1, 0.1 };

    return speed;
    // Ang_Lmot=
    // Ang_Rmot=
  }

  public void leftturn() {
    FR.set(0.1);
    BR.set(0.1);
    FL.set(-0.1);
    BL.set(-0.1);
  }

  public void rightturn() {
    FR.set(-0.2);
    BR.set(-0.2);
    FL.set(0.2);
    BL.set(0.2);
  }

  public void forward() {
    FR.set(0.2);
    BR.set(0.2);
    FL.set(0.2);
    BL.set(0.2);
  }

  public void back() {
    FR.set(-0.2);
    BR.set(-0.2);
    FL.set(-0.2);
    BL.set(-0.2);
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

  double a, b = 0;

  public void stepwisexy() {
    double totalDistance = (FL_encoder.getPosition() * (Math.PI *
        Units.inchesToMeters(6)) / 7.31)
        + ((BR_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6))) / 7.31);
    double distance = totalDistance / 2;
    double angle = -gyro.getAngle() % 360;
    // SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("distance", distance);
    SmartDashboard.putNumber("angle to take", angletotake);
    a = (double) Math.round(pose.getX() * 10) / 10.00;
    b = (double) Math.round(pose.getY() * 10) / 10.00;
    SmartDashboard.putNumber("x", a);
    SmartDashboard.putNumber("y", b);
    double fx = 0;
    double fy = 0;

    // angletotake=Math.toDegrees(Math.atan((fy-b)/(fx-a)));
    // angletotake=Math.toDegrees(Math.atan(1/1));
    angletotake = Math.toDegrees(Math.atan2((fy - b), (fx - a)));
    // angletotake = Math.toDegrees(Math.atan2(1, 1));
    // if(an)
    angletotake = (angletotake + 720) % 360;

    double d_theta = angletotake - angle;
    double theta_dir = d_theta / Math.abs(d_theta);

    if (Math.abs(d_theta) >= 180) {
      d_theta = 360 - Math.abs(d_theta);
      theta_dir *= -1;
    } else {
      d_theta = Math.abs(d_theta);
      theta_dir *= 1;
    }
    SmartDashboard.putNumber("angle to take with direction", d_theta * theta_dir);
    SmartDashboard.putNumber("D Theta", d_theta);
    SmartDashboard.putNumber("Dir", theta_dir);
    SmartDashboard.putNumber("angle", angle);
    // SmartDashboard.putNumber("angle to take", angletotake);
    // SmartDashboard.putNumber("angle to take", angletotake);
    // FR.set(speedcontrol(controller.calculate(distance, 1) -
    // controllerang.calculate(angle, angletotake)));
    // BR.set(speedcontrol(controller.calculate(distance, 1) -
    // controllerang.calculate(angle, angletotake)));
    // FL.set(speedcontrol(controller.calculate(distance, 1) +
    // controllerang.calculate(angle, angletotake)));
    // BL.set(speedcontrol(controller.calculate(distance, 1) +
    // controllerang.calculate(0, d_theta * theta_dir)));
    // FR.set(speedcontro(- controllerang.calculate(angle, angletotake)));
    // BR.set(speedcontro(- controllerang.calculate(angle, angletotake)));
    // FL.set(speedcontrol(controllerang.calculate(angle, angletotake)));
    // BL.set(speedcontrol(controllerang.calculate(angle, angletotake)));

    SmartDashboard.putNumber("angle to take", angletotake);
    SmartDashboard.putNumber("speed", controllerang.calculate(0, d_theta * theta_dir));

  }

  public void stepwise() {
    double totalDistance = (FL_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6)) / 10.71)
        + ((BR_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6))) / 10.71);
    double distance = totalDistance / 2;
    double angle = gyro.getAngle() % 360;
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("distance", distance);

    // SmartDashboard.putNumber("f", f);
    // SmartDashboard.putNumber("f1", f1);
    // SmartDashboard.putNumber("f2", f2);
    // SmartDashboard.putNumber("f3", f3);
    // SmartDashboard.putNumber("f4", f4);
    // SmartDashboard.putNumber("f5", f5);
    // SmartDashboard.putNumber("f6", f6);
    // SmartDashboard.putNumber("f7", f7);
    // SmartDashboard.putNumber("f8", f8);
    // SmartDashboard.putNumber("f9", f9);

    // // intake.set(-0.5);
    if (f == 1) {
      // intake.set(-0.5);
      if (distance < x[0] - dx) {
        FL.set(speedcontrol2(0)[0]);
        BL.set(speedcontrol2(0)[0]);
        FR.set(speedcontrol2(0)[1]);
        BR.set(speedcontrol2(0)[1]);
        // FR.set(speedcontrol(controller.calculate(distance, x[0]) -
        // controllerang.calculate(angle, y[0])));
        // BR.set(speedcontrol(controller.calculate(distance, x[0]) -
        // controllerang.calculate(angle, y[0])));
        // FL.set(speedcontrol(controller.calculate(distance, x[0]) +
        // controllerang.calculate(angle, y[0])));
        // BL.set(speedcontrol(controller.calculate(distance, x[0]) +
        // controllerang.calculate(angle, y[0])));
      } else {
        stop();
        // FR.set(-controllerang.calculate(angle, 0));
        // BR.set(-controllerang.calculate(angle, 0));
        // FL.set(controllerang.calculate(angle, 0));
        // BL.set(controllerang.calculate(angle, 0));
        controller.reset();
        controllerang.reset();
        controllerangle.reset();
        resetdistance();
        f = 0;
        f1 = 1;
      }
    }
    if (f1 == 1) {
      if (angle < y[1] - dy) {
        SmartDashboard.putNumber("2", angle);
        // rightturn();
        FR.set(speedcontrol(-controllerang.calculate(angle, y[1])));
        BR.set(speedcontrol(-controllerang.calculate(angle, y[1])));
        FL.set(speedcontrol(controllerang.calculate(angle, y[1])));
        BL.set(speedcontrol(controllerang.calculate(angle, y[1])));
        // SmartDashboard.putNumber("speed", controllerang.calculate(angle, 180));
        // SmartDashboard.putNumber("speed180-angle", 180 - angle);
      } else {
        stop();
        resetpid();
        resetdistance();
        controller.reset();
        controllerang.reset();
        SmartDashboard.putNumber("3", angle);
        f1 = 0;
        f2 = 1;
        // intake.set(0);
      }
    }
    if (f2 == 1) {
      if (distance < x[1] - dx) {
        FR.set(speedcontrol(controller.calculate(distance, x[1]) - controllerang.calculate(angle, y[1])));
        BR.set(speedcontrol(controller.calculate(distance, x[1]) - controllerang.calculate(angle, y[1])));
        FL.set(speedcontrol(controller.calculate(distance, x[1]) + controllerang.calculate(angle, y[1])));
        BL.set(speedcontrol(controller.calculate(distance, x[1]) + controllerang.calculate(angle, y[1])));
      } else {

        stop();

        controller.reset();
        resetdistance();
        resetpid();
        controller.reset();
        controllerang.reset();
        f2 = 0;
        f3 = 1;
        // SmartDashboard.putNumber("hellof2", angle);
      }
    }
    if (f3 == 1) {
      if (angle > y[2] - dy) {
        // SmartDashboard.putNumber("2", angle);
        // rightturn();
        SmartDashboard.putNumber("rspeed",
            (-controllerang.calculate(angle, y[2])));
        SmartDashboard.putNumber("lspeed",
            (controllerang.calculate(angle, y[2])));
        FR.set(speedcontrol(-controllerang.calculate(angle, y[2])));
        BR.set(speedcontrol(-controllerang.calculate(angle, y[2])));
        FL.set(speedcontrol(controllerang.calculate(angle, y[2])));
        BL.set(speedcontrol(controllerang.calculate(angle, y[2])));
        // SmartDashboard.putNumber("speed", controllerang.calculate(angle, 180));
        // SmartDashboard.putNumber("speed180-angle", 180 - angle);
      } else {
        stop();
        resetpid();
        resetdistance();
        controller.reset();
        controllerang.reset();
        SmartDashboard.putNumber("3", angle);
        f3 = 0;
        f4 = 1;
        // intake.set(0)
      }
    }
    if (f4 == 1) {
      SmartDashboard.putNumber("rspeed",
          (controller.calculate(distance, x[2]) - controllerang.calculate(angle, y[2])));
      SmartDashboard.putNumber("lspeed",
          (controller.calculate(distance, x[2]) + controllerang.calculate(angle, y[2])));
      if (distance < x[2] + dx) {
        FR.set(speedcontrol(controller.calculate(distance, x[2]) - controllerang.calculate(angle, y[2])));
        BR.set(speedcontrol(controller.calculate(distance, x[2]) - controllerang.calculate(angle, y[2])));
        FL.set(speedcontrol(controller.calculate(distance, x[2]) + controllerang.calculate(angle, y[2])));
        BL.set(speedcontrol(controller.calculate(distance, x[2]) + controllerang.calculate(angle, y[2])));
      } else {

        stop();

        controller.reset();
        resetdistance();
        resetpid();
        controller.reset();
        controllerang.reset();
        f4 = 0;
        f5 = 1;
        // SmartDashboard.putNumber("hellof2", angle);
      }
    }
    if (f5 == 1) {
      if (angle < y[3] - dy) {
        // SmartDashboard.putNumber("2", angle);
        // rightturn();
        FR.set(speedcontrol(-controllerang.calculate(angle, y[3])));
        BR.set(speedcontrol(-controllerang.calculate(angle, y[3])));
        FL.set(speedcontrol(controllerang.calculate(angle, y[3])));
        BL.set(speedcontrol(controllerang.calculate(angle, y[3])));
        // SmartDashboard.putNumber("speed", controllerang.calculate(angle, 180));
        // SmartDashboard.putNumber("speed180-angle", 180 - angle);
      } else {
        stop();
        resetpid();
        resetdistance();
        controller.reset();
        controllerang.reset();
        SmartDashboard.putNumber("3", angle);
        f5 = 0;
        f6 = 1;
        // intake.set(0)
      }
    }
    if (f6 == 1) {
      if (distance < x[2] - dx) {
        FR.set(speedcontrol(controller.calculate(distance, x[2]) - controllerang.calculate(angle, y[3])));
        BR.set(speedcontrol(controller.calculate(distance, x[2]) - controllerang.calculate(angle, y[3])));
        FL.set(speedcontrol(controller.calculate(distance, x[2]) + controllerang.calculate(angle, y[3])));
        BL.set(speedcontrol(controller.calculate(distance, x[2]) + controllerang.calculate(angle, y[3])));
      } else {

        stop();

        controller.reset();
        resetdistance();
        resetpid();
        controller.reset();
        controllerang.reset();
        f6 = 0;
        f7 = 1;
        // SmartDashboard.putNumber("hellof2", angle);
      }
    }
    // if (f7 == 1) {
    // if (angle < y[4] - dy) {
    // // SmartDashboard.putNumber("2", angle);
    // // rightturn();
    // FR.set(speedcontrol(-controllerang.calculate(angle, y[4])));
    // BR.set(speedcontrol(-controllerang.calculate(angle, y[4])));
    // FL.set(speedcontrol(controllerang.calculate(angle, y[4])));
    // BL.set(speedcontrol(controllerang.calculate(angle, y[4])));
    // // SmartDashboard.putNumber("speed", controllerang.calculate(angle, 180));
    // // SmartDashboard.putNumber("speed180-angle", 180 - angle);
    // } else {
    // stop();
    // resetpid();
    // resetdistance();
    // controller.reset();
    // controllerang.reset();
    // SmartDashboard.putNumber("3", angle);
    // f7 = 0;
    // f8 = 1;
    // // intake.set(0)
    // }
    // }
    // if (f8== 1) {
    // if (distance < x[2]-dx) {
    // FR.set(speedcontrol(controller.calculate(distance, x[2]) -
    // controllerang.calculate(angle, y[4])));
    // BR.set(speedcontrol(controller.calculate(distance, x[2]) -
    // controllerang.calculate(angle, y[4])));
    // FL.set(speedcontrol(controller.calculate(distance, x[2]) +
    // controllerang.calculate(angle, y[4])));
    // BL.set(speedcontrol(controller.calculate(distance, x[2]) +
    // controllerang.calculate(angle, y[4])));
    // } else {

    // stop();

    // controller.reset();
    // resetdistance();
    // resetpid();
    // controller.reset();
    // controllerang.reset();
    // f8 = 0;
    // f9 = 1;
    // // SmartDashboard.putNumber("hellof2", angle);
    // }
    // }
    if (f7 == 1) {
      stop();
      f7 = 0;
      // FR.set(-controllerangle.calculate(angle, 0));
      // BR.set(-controllerangle.calculate(angle, 0));
      // FL.set(controllerangle.calculate(angle, 0));
      // BL.set(controllerangle.calculate(angle, 0));
    }
  }

  double ps1 = 0;
  double ps2 = 0;

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // stepwisexy();
    double totalDistance = (FL_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6)) / 10.71)
        + ((BR_encoder.getPosition() * (Math.PI * Units.inchesToMeters(6))) / 10.71);
    double distance = totalDistance / 2;
    // double a123 = gyro.getAngle() % 360;
    // SmartDashboard.putNumber("a123", a123);
    SmartDashboard.putNumber("distance", distance);
    // stepwisexy();
    // // double dis = left.getSelectedSensorPosition();
    // // left.set(0.8);
    // SmartDashboard.putNumber("R",
    // controller.calculate(distance,1)-controllerang.calculate(angle,0));
    // SmartDashboard.putNumber("L",
    // controller.calculate(distance,1)+controllerang.calculate(angle,0));
    // double error = (dis - 3000) * 0.005;
    // SmartDashboard.putNumber("distance", dis);
    if (joy1.getRawButton(1)) {
      // left.set(0.1);
      // FR.set(speedcontrol(controller.calculate(distance, 1) -
      // controllerang.calculate(a123, 0)));
      // BR.set(speedcontrol(controller.calculate(distance, 1) -
      // controllerang.calculate(a123, 0)));
      // FL.set(speedcontrol(controller.calculate(distance, 1) +
      // controllerang.calculate(a123, 0)));
      // BL.set(speedcontrol(controller.calculate(distance, 1) +
      // controllerang.calculate(a123, 0)));
      // FR.set(speedcontrol(-controllerang.calculate(a123,180)));
      // BR.set(speedcontrol(-controllerang.calculate(a123,180)));
      // FL.set(speedcontrol(controllerang.calculate(a123,180)));
      // BL.set(speedcontrol(controllerang.calculate(a123,180)));
      // stepwise();
      //
      double s1 = speedcontrol2(0)[0];
      double s2 = speedcontrol2(0)[1];
      FL.set(s1);
      BL.set(s1);
      FR.set(s2);
      BR.set(s2);
      ps1 = s1;
      ps2 = s2;
      // shooter.set(-0.3);
      // SmartDashboard.putNumber("speed of left motor",
      // controllerangle.calculate(distance,2));
      SmartDashboard.putNumber("Rspeeds2", s2);
      SmartDashboard.putNumber("Lspeeds1", s1);
      // left.set(controllerangle.calculate(distance,2));
    }
    if (joy1.getRawButton(2)) {
      // shooter.set(0);
      // }
      // left.set(error);

      // limelightcode****************************************************************************************************8
      // NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      // NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
      // NetworkTableEntry tx = table.getEntry("tx");
      // NetworkTableEntry ty = table.getEntry("ty");
      // NetworkTableEntry ta = table.getEntry("ta");

      // // read values periodically
      // double x = tx.getDouble(0.0);
      // double y = ty.getDouble(0.0);
      // double area = ta.getDouble(0.0);

      // // post to smart dashboard periodically
      // SmartDashboard.putNumber("LimelightX", x);
      // SmartDashboard.putNumber("LimelightY", y);
      // SmartDashboard.putNumber("LimelightArea", area);

      // FR.set(-pidlimelight.calculate(x, 0));
      // BR.set(-pidlimelight.calculate(x, 0));
      // FL.set(pidlimelight.calculate(x, 0));
      // BL.set(pidlimelight.calculate(x, 0));
    }
    if (joy1.getRawButton(3)) {
      // forward();
    }
    if (joy1.getRawButton(4)) {
      // rightturn();
    }
    // idk***************************************************************************************************************
    // NetworkTable table = NetworkTable.getTable("limelight");
    // double targetOffsetAngle_Horizontal = table.getNumber("tx", 0);
    // double targetOffsetAngle_Vertical = table.getNumber("ty", 0);
    // double targetArea = table.getNumber("ta", 0);
    // double targetSkew = table.getNumber("ts", 0);
    // ******************************************************************************************************************
    // // if (joy1.getRawButton(1)) {
    // // setpoint = 1000; //1000==180degree
    // // } else if (joy1.getRawButton(2)) {
    // // setpoint = 0;
    // // }
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
