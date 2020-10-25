package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class VoyagerBot {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;

    public Servo wobbleArm = null;
    public Servo wobbleClaw = null;
    /*
    public Servo claw = null;
    public Servo back = null;
    public Servo skystone = null;
    public DcMotor lift = null;
    public DcMotor yeeter = null;
     */
    HardwareMap hwMap = null;
    /*
    public CRServo extension = null;
    public Servo back2 = null;
    public Servo gripper = null;
    public ColorSensor color_front = null;
    public ColorSensor color_back = null;
     */
    public RevBlinkinLedDriver underglow = null;
    public Rev2mDistanceSensor distance = null;
    BNO055IMU imu = null;
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_RADIUS_INCHES = 10 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * Math.PI);
    public VoyagerBot() {

    }
    public void resetEncoders() {
        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void newReset() {
        this.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        leftFront = hwMap.get(DcMotor.class, "front_left");
        leftBack = hwMap.get(DcMotor.class, "back_left");
        rightFront = hwMap.get(DcMotor.class, "front_right");
        rightBack = hwMap.get(DcMotor.class, "back_right");
        wobbleArm = hwMap.get(Servo.class, "wobble_arm");
        wobbleClaw = hwMap.get(Servo.class, "wobble_claw");
        /*
        yeeter = hwMap.get(DcMotor.class, "yeeter");
        claw = hwMap.get(Servo.class, "claw");
        back = hwMap.get(Servo.class, "back");
        gripper = hwMap.get(Servo.class, "gripper");
        lift = hwMap.get(DcMotor.class, "lift_motor");
        extension = hwMap.get(CRServo.class, "extension");
        //extension = hwMap.get(Servo.class, "extension");
        back2 = hwMap.get(Servo.class, "back2");
        skystone = hwMap.get(Servo.class, "stone");
         */
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        /*
        color_front = hwMap.get(ColorSensor.class, "color_front");
        color_back = hwMap.get(ColorSensor.class, "color_back");
        color_front.enableLed(false);
        color_back.enableLed(false);
         */
        distance = hwMap.get(Rev2mDistanceSensor.class, "distance");
        underglow = hwMap.get(RevBlinkinLedDriver.class, "underglow");
        underglow.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED);


        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        //yeeter.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        wobbleArm.setPosition(0);
        wobbleClaw.setPosition(0);
        /*
        claw.setPosition(0);
        back.setPosition(0.35);
        back2.setPosition(0.65);
        extension.setPower(0);
        extension.setPower(0);
        gripper.setPosition(0.1);
        skystone.setPosition(0.4);
         */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}