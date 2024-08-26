// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkFlex;

/** 
 * Controls the shooting mechanism
 */

 public class Shooter {

    private final int SHOOTER_MOTOR_1_CAN = 21;
    private final int SHOOTER_MOTOR_2_CAN = 22;
    private final int MOTOR_CURRENT_LIMIT = 40;
    private final double DUMP_SHOT_POWER  = 0.1;
    private final double SHOT_POWER  = .5;   // Competition speed: 1, Demo speed: 0.35
    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;
    private Grabber instancedGrabber;

    public Shooter(){
        // Instantiate shooter motors
        shooterMotor1 = new CANSparkFlex(SHOOTER_MOTOR_1_CAN, MotorType.kBrushless);
        shooterMotor1.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        shooterMotor1.setIdleMode(IdleMode.kCoast);
        
        shooterMotor2 = new CANSparkFlex(SHOOTER_MOTOR_2_CAN, MotorType.kBrushless);
        shooterMotor2.setSmartCurrentLimit(MOTOR_CURRENT_LIMIT);
        shooterMotor2.setIdleMode(IdleMode.kCoast);
        shooterMotor2.setInverted(true);

        // Get the same grabber instance
        instancedGrabber = Grabber.getInstance();

        // Spin the motors
        //spinup();
    }

    /**
     * Set power to spin motors, might not be needed if there's only two power values to use 
     * @param shootPower The power to shoot at
     */
    public void startShooting(double shootPower) {
        shooterMotor1.set(MathUtil.clamp(shootPower, -1.0, 1.0));
        shooterMotor2.set(MathUtil.clamp(shootPower, -1.0, 1.0));
        //instancedGrabber.intakeOutake(true, false);
        instancedGrabber.setMotorPower(instancedGrabber.FEED_POWER);
    }

    /**
     * <p> Turns off shooter motor
     */
    public void stopShooting() {
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

    /**
     * Spinup motor, to be ready at all times
     */
    public void spinup() {
        shooterMotor1.set(SHOT_POWER);
        shooterMotor2.set(SHOT_POWER);
    }

    /**
     * Spindown motor to 0 power
     */
    public void spindown() {
        shooterMotor1.set(0.0);
        shooterMotor2.set(0.0);
    }

    /**********************************************************************
     * 
     *    TEST FUNCTIONS
     * 
     **********************************************************************/
    public void testSpin() {
        shooterMotor2.set(0.05);
    }

}
