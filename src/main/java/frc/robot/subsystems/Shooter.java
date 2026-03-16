
package frc.robot.subsystems;


import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;


public class Shooter {
    private static TalonFX shooter1;
    private TalonFX shooter2;
    private static TalonFX preshooter;
    private static TalonFX feeder;

    private TalonSRX hoodSrx1;
    private TalonSRX hoodSrx2;
    private int hoodMode = 0; // 0 = min, 1 = mid, 2 = max
    private double curPosition = 0;
    private AnalogInput m_analog = new AnalogInput(1);
    private AnalogInput m2_analog = new AnalogInput(2);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    
    private InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();
    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("bigboy");
    double angleToGoalRadians = Math.toRadians(25  + targetOffsetAngle_Vertical);
    double tx = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getX();
    double tz = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getZ();
    double distance = Math.sqrt((tx* tx) +  (tz * tz)) * 3.28;
    double g = 32.2;                      // ft/s^2
    double h0 = 29.0 / 12.0;              // shooter height (ft)
    double ht = 6.0;                      // hoop height (ft)
    double deltaH = ht - h0;              // height difference (ft)
    double theta = Math.toRadians(55);    // hood angle (radians)
    double r = 4.0 / 12.0;                // wheel radius (ft)

    


    public Shooter(int CanID1, int CanID2, int CanID3, int CanID4, int CanID5, int CanID6){
        //canbus = new CANBus("rio");
        TalonFX s1 = new TalonFX(CanID1);
        TalonFX s2 = new TalonFX(CanID2);
        TalonFX f = new TalonFX(CanID3);
        TalonFX ps = new TalonFX(CanID4);
        TalonSRX s5 = new TalonSRX(CanID5);
        TalonSRX s6 = new TalonSRX(CanID6);
        shooter1 = s1;
        shooter2 = s2;
        shooter1.setNeutralMode(NeutralModeValue.Coast);
        shooter2.setNeutralMode(NeutralModeValue.Coast);
        feeder = f;
        preshooter = ps;
        hoodSrx1 = s5;
        hoodSrx2 = s6;
        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 4;
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kG = 0.5; //.3
        slot0Configs.kS = 0.0; //.00
        slot0Configs.kV = 0.3; // 0.01
        slot0Configs.kA = 0.3; //0.1
        slot0Configs.kP = 0.75; //4.8
        slot0Configs.kI = 0.2; //00
        slot0Configs.kD = 0.1;
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.5; //.12
        motionMagicConfigs.MotionMagicExpo_kA = 0.6; //.1


        // Apply configurations to the motor
        shooter1.getConfigurator().apply(talonFXConfigs);
        shooter2.getConfigurator().apply(talonFXConfigs);
        
        shooterMap.put(3.0, 45.0);
        shooterMap.put(5.0, 60.0);
        shooterMap.put(7.0, 75.0);
        // shooterMap.put(3.0, 10.0);
        // shooterMap.put(5.0, 20.0);
        // shooterMap.put(7.0, 30.0);

    }
    


    public double getRPS() {
        return shooterMap.get(distance);
    }

    public void sSetSpeed(double RPS){
        //setting the motor to reach a speed (rps) from imap
        //NOTE: Imap MUST be in RPS to retrieve proper values
        shooter1.setControl(new VelocityVoltage(RPS));
        shooter2.setControl(new VelocityVoltage(-RPS));
        
    }
    
    public double getSpeed(){
        //this returns the speed of the motor in rotations per second
        return shooter1.getVelocity().getValueAsDouble();
        
    }
    public void stopShooter(){
        shooter1.setControl(new NeutralOut());
        shooter2.setControl(new NeutralOut());
    }
    public void shoot(){
        double targetVelocity = getRPS();
        sSetSpeed(targetVelocity);
        if (getSpeed() >= targetVelocity){ 
            fSetSpeed(.5);
            pSetSpeed(.75);
        }
    }

    public void fSetSpeed(double percent){
        feeder.set(-percent);
    }
    public void pSetSpeed(double percent){
        preshooter.set(percent);
    }
    public double getDistance(){
        double tx = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getX();
        double tz = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getZ();
        return Math.sqrt((tx* tx) +  (tz * tz)) * 3.28;

    }

    public double getVoltage1(){
        return (Math.round(m_analog.getVoltage()*10))/10.0;
    }
    public double getVoltage2(){
        return (Math.round(m2_analog.getVoltage()*10))/10.0;
    }
    public void stopHood(){
        hoodSrx1.set(ControlMode.PercentOutput, 0);
    }

    public void moveHood( double speed){
            hoodSrx1.set(ControlMode.PercentOutput, speed);
            hoodSrx2.set(ControlMode.PercentOutput, -speed);
    }

    
}

