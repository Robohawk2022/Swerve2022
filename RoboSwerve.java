package frc.robot;

import javax.sound.midi.SysexMessage;
import javax.swing.plaf.basic.BasicBorders.SplitPaneBorder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.SuperJoystick;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
public class RoboSwerve {
    /**
     * @name RoboSwerve Init
     * @version 2.0
     * @author Mac Lawson
     * Documentation located in GitBooks
     */
    public static void SwerveInit() {
        final int  FLangleID = 8;
        final int FLdriveID = 7;
        final int  FRangleID = 6;
        final int FRdriveID = 5;
        final int  BRangleID = 4;
        final int BRdriveID = 3;
        final int  BLangleID = 2;
        final int BLdriveID = 1;
        CANSparkMax FLangleMotor;
        CANSparkMax FLdriveMotor;
        CANSparkMax FRdriveMotor;
        CANSparkMax FRangleMotor;
        CANSparkMax BLangleMotor;
        CANSparkMax BLdriveMotor;
        CANSparkMax BRdriveMotor;
        CANSparkMax BRangleMotor;
        SparkMaxPIDController m_PIDController1;
        SparkMaxPIDController m_PIDController2;
        SparkMaxPIDController m_PIDController3;
        SparkMaxPIDController m_PIDController4;
        RelativeEncoder m_encoder1;
        RelativeEncoder m_encoder2;
        RelativeEncoder m_encoder3;
        RelativeEncoder m_encoder4;
        XboxController drive_control;
        Timer autoTimer;
        double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
        drive_control = new XboxController(0);
        // initialize motor
        FLangleMotor = new CANSparkMax(FLangleID, MotorType.kBrushless);
        FLdriveMotor = new CANSparkMax(FLdriveID, MotorType.kBrushless);
        FRangleMotor = new CANSparkMax(FRangleID, MotorType.kBrushless);
        FRdriveMotor = new CANSparkMax(FRdriveID, MotorType.kBrushless);
        BRangleMotor = new CANSparkMax(BRangleID, MotorType.kBrushless);
        BRdriveMotor = new CANSparkMax(BRdriveID, MotorType.kBrushless);
        BLdriveMotor = new CANSparkMax(BLdriveID, MotorType.kBrushless);
        BLangleMotor = new CANSparkMax(BLangleID, MotorType.kBrushless);
    
        m_PIDController1 = FLangleMotor.getPIDController();
        m_PIDController2 = FRangleMotor.getPIDController();
        m_PIDController3 = BRangleMotor.getPIDController();
        m_PIDController4 = BLangleMotor.getPIDController();
    
        m_encoder1 = FLangleMotor.getEncoder();
        m_encoder2 = FRangleMotor.getEncoder();
        m_encoder3 = BRangleMotor.getEncoder();
        m_encoder4 = BLangleMotor.getEncoder();
    
    
        // PID coefficients
        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
    
        // set PID coefficients
        m_PIDController1.setP(kP);
        m_PIDController2.setP(kP);
        m_PIDController3.setP(kP);
        m_PIDController4.setP(kP);
    
        m_PIDController1.setI(kI);
        m_PIDController2.setI(kI);
        m_PIDController3.setI(kI);
        m_PIDController4.setI(kI);
    
        m_PIDController1.setD(kD);
        m_PIDController2.setD(kD);
        m_PIDController3.setD(kD);
        m_PIDController4.setD(kD);
    
        m_PIDController1.setIZone(kIz);
        m_PIDController2.setIZone(kIz);
        m_PIDController3.setIZone(kIz);
        m_PIDController4.setIZone(kIz);
    
        m_PIDController1.setFF(kFF);
        m_PIDController2.setFF(kFF);
        m_PIDController3.setFF(kFF);
        m_PIDController4.setFF(kFF);
    
        m_PIDController1.setOutputRange(kMinOutput, kMaxOutput);
        m_PIDController2.setOutputRange(kMinOutput, kMaxOutput);
        m_PIDController3.setOutputRange(kMinOutput, kMaxOutput);
        m_PIDController4.setOutputRange(kMinOutput, kMaxOutput);
    
        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        SmartDashboard.putNumber("Set Rotations", 0);
        SmartDashboard.putNumber("Run Speed Motor?", 0);
        SmartDashboard.putBoolean("Joystick Control", false);
    }
    public static void BasicDrive() {
        if(drive_control.getLeftY() > 0) {
            FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
            FRdriveMotor.set((drive_control.getLeftY() * -1) / 4);
            BRdriveMotor.set(drive_control.getLeftY() / 4);
            BLdriveMotor.set(drive_control.getLeftY() / 4);
    
            m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
            m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
            m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);
            m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / -2), CANSparkMax.ControlType.kPosition);            
          }
        if(drive_control.getLeftY() < 0) {
            FLdriveMotor.set((drive_control.getLeftY() * -1) / 4);
            FRdriveMotor.set((drive_control.getLeftY() * -1) / 4);
            BRdriveMotor.set(drive_control.getLeftY() / 4);
            BLdriveMotor.set(drive_control.getLeftY() / 4);
    
            m_PIDController1.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
            m_PIDController2.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
            m_PIDController3.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);
            m_PIDController4.setReference(((drive_control.getRawAxis(0) * -10) / 2), CANSparkMax.ControlType.kPosition);   
          }
    }
    public static void RotationalSwerve() {
        if(drive_control.getRightX() > .005) {
          m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set(.25);
          m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
          FRdriveMotor.set(-.25);
          m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
          BLdriveMotor.set(-.25);
          m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
          BRdriveMotor.set(.25);
        }
           // Rotation
        if(drive_control.getRightX() < -.005) {
          m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set(-.25);
          m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
          FRdriveMotor.set(.25);
          m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
          BLdriveMotor.set(.25);
          m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
          BRdriveMotor.set(-.25);    
         }
      }     
    public static void StrafeSwerve() {
        if(drive_control.getRawAxis(0) == 1) {
          m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set(- .25);
          m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
          FRdriveMotor.set(- .25);
          m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
          BLdriveMotor.set(.25);
          m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
          BRdriveMotor.set(.25);
        }
        if(drive_control.getRawAxis(0) == -1) {
          m_PIDController1.setReference(4,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set(.25);
          m_PIDController2.setReference(4,  CANSparkMax.ControlType.kPosition);
          FRdriveMotor.set(.25);
          m_PIDController3.setReference(4,  CANSparkMax.ControlType.kPosition);
          BLdriveMotor.set(- .25);
          m_PIDController4.setReference(4,  CANSparkMax.ControlType.kPosition);
          BRdriveMotor.set(- .25);
        }    
      }    
    public static void SnakeDrive() {
        if(drive_control.getRightTriggerAxis() > .05) {
          m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
          m_PIDController2.setReference(-2,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set((drive_control.getLeftY() * -1));
          FRdriveMotor.set((drive_control.getLeftY() * -1));
          BRdriveMotor.set(drive_control.getLeftY());
          BLdriveMotor.set(drive_control.getLeftY());
        }
        if(drive_control.getLeftTriggerAxis() > .05) {
          m_PIDController1.setReference(2,  CANSparkMax.ControlType.kPosition);
          m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set((drive_control.getLeftY() * -1));
          FRdriveMotor.set((drive_control.getLeftY() * -1));
          BRdriveMotor.set(drive_control.getLeftY());
          BLdriveMotor.set(drive_control.getLeftY());
        }
      }    
    public static void AimBot() {
        if(drive_control.getLeftBumper() == true) {
          m_PIDController1.setReference(-2,  CANSparkMax.ControlType.kPosition);
          m_PIDController2.setReference(2,  CANSparkMax.ControlType.kPosition);
          m_PIDController3.setReference(-2,  CANSparkMax.ControlType.kPosition);
          m_PIDController4.setReference(2,  CANSparkMax.ControlType.kPosition);
          FLdriveMotor.set(drive_control.getRightX() / 15);
          FRdriveMotor.set((drive_control.getRightX() * -1) / 15);
          BRdriveMotor.set(drive_control.getRightX() / 15);
          BLdriveMotor.set((drive_control.getRightX() * -1) / 15);
        }
      }
}
