package org.firstinspires.ftc.teamcode.Template;


import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// Template for field centric Mecanum wheel drive




@Config
@TeleOp
public class Telop_Template extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();  //delete final if prob
    DcMotor RightFrontDrive;
    DcMotor LeftFrontDrive;
    DcMotor RightBackDrive;
    DcMotor LeftBackDrive;

    DcMotor LiftMotor;

    Servo RightServo;
    Servo LeftServo;

    BNO055IMU BNO055;
    Orientation angles;




    @Override
    public void runOpMode() {

        //TODO: Make sure you have the motor Config correctly on the driver hub

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        BNO055 = hardwareMap.get(BNO055IMU.class, "IMU");
        BNO055.initialize(parameters);

        RightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        LeftFrontDrive = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        RightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        LeftBackDrive = hardwareMap.get(DcMotor.class, "LeftBackDrive");

        LiftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");


        RightServo = hardwareMap.get(Servo.class, "RightServo");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");

        RightServo.setPosition(.35);
        LeftServo.setPosition(.65);



        RightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        LeftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);



        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            angles = BNO055.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            float pi = (float) 3.14159;
            float gyroRadians = angles.firstAngle * pi / 180;

            float axial2 = (float) (axial * cos(gyroRadians) - lateral * sin(gyroRadians));
            lateral = axial * sin(gyroRadians) + lateral * cos(gyroRadians);
            axial = axial2;

            double leftFrontSign  = Math.signum((axial + lateral + yaw));
            double rightFrontSign = Math.signum((axial - lateral - yaw));
            double leftBackSign   = Math.signum((axial - lateral + yaw));
            double rightBackSign  = Math.signum((axial + lateral - yaw));

            double leftFrontPower  = /*leftFrontSign */ pow((axial + lateral + yaw), 3);
            double rightFrontPower = /*rightFrontSign */ pow((axial - lateral - yaw), 3);
            double leftBackPower   = /*leftBackSign */ pow((axial - lateral + yaw), 3);
            double rightBackPower  = /*rightBackSign */ pow((axial + lateral - yaw), 3);


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            LeftFrontDrive.setPower(leftFrontPower * 3/4);
            RightFrontDrive.setPower(rightFrontPower * 3/4);
            LeftBackDrive.setPower(leftBackPower* 3/4);
            RightBackDrive.setPower(rightBackPower * 3/4 );




            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("YAW", angles.firstAngle);
            telemetry.update();


        }
    }

}