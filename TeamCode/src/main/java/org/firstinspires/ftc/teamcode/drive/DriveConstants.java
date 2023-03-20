 package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class DriveConstants {

// TODO: Change the values of TICKS_PER_REV & MAX_RPM to the values of your motors for your drive chassis
    public static final double TICKS_PER_REV = 384.5;
    public static final double MAX_RPM = 435;


    public static final boolean RUN_USING_ENCODER = true;  // Keep this value true since you will be using the encoders on the motor but if your using dead wheels set it to false.


    // Change MOTOR_VELO_PID if you will be using the PID. PID will help prevent the robot to drift; making robot movement more accurate.
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(15, .5, 12,   //Increase the P value first while the I and D is at zero
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    //TODO: Change WHEEL_RADIUS, GEAR_RATIO, and TRACK_WIDTH base on your robot
    public static double WHEEL_RADIUS = 1.8898; // in
    public static double GEAR_RATIO = (1 * 0.96443093); // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 13.04 ; // in

// these values can be leaved alone.   For more information read the Roadrunner doc.
    public static double kV = 1 / rpmToVelocity(MAX_RPM);
    public static double kA = 0 ;
    public static double kStatic = 0 ;

// These values are the max for robot speed.
    // Increase or decrease the values to make the robot to be faster or slower
    public static double MAX_VEL =  60;
    public static double MAX_ACCEL = 60;

    // Turn speed
    public static double MAX_ANG_VEL = Math.toRadians(180);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

// Line 41-53 can be left alone
    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
         return 32767 / ticksPerSecond;
    }
}
