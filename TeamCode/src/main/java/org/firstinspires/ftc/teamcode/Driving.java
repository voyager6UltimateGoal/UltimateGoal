package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Driving {
    private VoyagerBot robot = new VoyagerBot();
    private LinearOpMode opmode;
    static final double COUNTS_PER_MOTOR_REV = 134.4;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_RADIUS_INCHES = 10 / 2.54;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_RADIUS_INCHES * Math.PI);
    private boolean targetVisible = false;
    private OpenGLMatrix lastLocation = null;
    //Detecting detector = new Detecting();
    Orientation lastAngles = new Orientation();
    private ElapsedTime runtime = new ElapsedTime();
    double globalAngle, correction;
    boolean armDown = false;
    float global_angle = 0;
    public Driving(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void resetEncoders() {
        robot.resetEncoders();
    }
    public void initHwMap() {
        robot.init(opmode.hardwareMap);
        opmode.telemetry.addData("[!]", "Wait for camera initialization...");
        opmode.telemetry.update();
        //detector.init(opmode.hardwareMap);
    }

    public void drive(double speed, double inches) {
        inches = inches * 11.6 / 59;
        int leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget;
        if(opmode.opModeIsActive()) {
            leftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition() +  (int)(inches * COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);
            if(inches >= 0) {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() <= leftFrontTarget | robot.leftBack.getCurrentPosition() <= leftBackTarget | robot.rightFront.getCurrentPosition() <= rightFrontTarget | robot.rightBack.getCurrentPosition() <= rightBackTarget)) {

                }
            } else {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() >= leftFrontTarget | robot.leftBack.getCurrentPosition() >= leftBackTarget | robot.rightFront.getCurrentPosition() >= rightFrontTarget | robot.rightBack.getCurrentPosition() >= rightBackTarget)) {

                }
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
    }

    public float getError(float target) {
        Orientation angles;
        float heading, robotError;
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        robotError = target - heading;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(float error, double coef) {
        return Range.clip(coef * error, -1, 1);
    }
    public void gyrodrive(double speed, double inches) {
        inches = inches * 24 / 59;
        int leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget;
        double steer;
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        if(opmode.opModeIsActive()) {
            leftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition() +  (int)(inches * COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(speed);
            if(inches >= 0) {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() <= leftFrontTarget | robot.leftBack.getCurrentPosition() <= leftBackTarget | robot.rightFront.getCurrentPosition() <= rightFrontTarget | robot.rightBack.getCurrentPosition() <= rightBackTarget)) {
                    steer = getSteer(getError(global_angle), 0.02);
                    leftFrontPower = speed - steer;
                    leftBackPower = speed - steer;
                    rightFrontPower = speed + steer;
                    rightBackPower = speed + steer;
                    robot.leftFront.setPower(leftFrontPower);
                    robot.rightFront.setPower(rightFrontPower);
                    robot.leftBack.setPower(leftBackPower);
                    robot.rightBack.setPower(rightBackPower);
                }
            } else {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() >= leftFrontTarget | robot.leftBack.getCurrentPosition() >= leftBackTarget | robot.rightFront.getCurrentPosition() >= rightFrontTarget | robot.rightBack.getCurrentPosition() >= rightBackTarget)) {
                    steer = getSteer(getError(global_angle), 0.02);
                    leftFrontPower = speed - steer;
                    leftBackPower = speed - steer;
                    rightFrontPower = speed + steer;
                    rightBackPower = speed + steer;
                    robot.leftFront.setPower(leftFrontPower);
                    robot.rightFront.setPower(rightFrontPower);
                    robot.leftBack.setPower(leftBackPower);
                    robot.rightBack.setPower(rightBackPower);
                }
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
    }
    public void gyrostrafe(double speed, double inches) {
        inches = inches * 24 / 55;
        int leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget;
        double steer;
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;
        if(opmode.opModeIsActive()) {
            leftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition() -  (int)(inches * COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(-speed);
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);

            if(inches >= 0) {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() <= leftFrontTarget | robot.leftBack.getCurrentPosition() >= leftBackTarget | robot.rightFront.getCurrentPosition() >= rightFrontTarget | robot.rightBack.getCurrentPosition() <= rightBackTarget)) {
                    steer = getSteer(getError(global_angle), 0.05);
                    leftFrontPower = speed - steer;
                    leftBackPower = -speed - steer;
                    rightFrontPower = -speed + steer;
                    rightBackPower = speed + steer;
                    robot.leftFront.setPower(leftFrontPower);
                    robot.rightFront.setPower(rightFrontPower);
                    robot.leftBack.setPower(leftBackPower);
                    robot.rightBack.setPower(rightBackPower);
                }
            } else {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() >= leftFrontTarget | robot.leftBack.getCurrentPosition() <= leftBackTarget | robot.rightFront.getCurrentPosition() <= rightFrontTarget | robot.rightBack.getCurrentPosition() >= rightBackTarget)) {
                    steer = -getSteer(getError(global_angle), 0.05);
                    leftFrontPower = speed + steer;
                    leftBackPower = -speed - steer;
                    rightFrontPower = -speed + steer;
                    rightBackPower = speed - steer;
                    robot.leftFront.setPower(leftFrontPower);
                    robot.rightFront.setPower(rightFrontPower);
                    robot.leftBack.setPower(leftBackPower);
                    robot.rightBack.setPower(rightBackPower);
                }
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
    }

    public void strafe(double speed, double inches) {
        inches = inches * 13.7 / 59;
        int leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget;
        if(opmode.opModeIsActive()) {
            leftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition() -  (int)(inches * COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(-speed);
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);
            if(inches >= 0) {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() <= leftFrontTarget | robot.leftBack.getCurrentPosition() >= leftBackTarget | robot.rightFront.getCurrentPosition() >= rightFrontTarget | robot.rightBack.getCurrentPosition() <= rightBackTarget)) {

                }
            } else {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() >= leftFrontTarget | robot.leftBack.getCurrentPosition() <= leftBackTarget | robot.rightFront.getCurrentPosition() <= rightFrontTarget | robot.rightBack.getCurrentPosition() >= rightBackTarget)) {

                }
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
    }
    /*
    public void strafeTillLimit(double speed, double _inches) {
        double inches = Math.abs(_inches);
        if(opmode.opModeIsActive()) {
            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(-speed);
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);
            while(opmode.opModeIsActive() && (robot.distance.getDistance(DistanceUnit.INCH) > inches)) {
                opmode.telemetry.addData("Distance", robot.distance.getDistance(DistanceUnit.INCH));
                opmode.telemetry.update();
            }
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
        }
    }
    public void startAngles() {
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }
    public float getAngle() {
        Orientation angles;
        float heading;
        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        return heading;
    }
    public void correct() {
        float offset = getAngle();
        if(1 == 1) {
            if(offset > 2) {
                gyroturn(0.1, 2);
            } else if(offset < -2) {
                gyroturn(-0.1, -2);
            }
        }
    }
    */
    public void gyroturn(double speed, double degrees) {
        Orientation angles;
        float heading;
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
        if(opmode.opModeIsActive()) {
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.leftFront.setPower(speed);
            robot.rightFront.setPower(-speed);
            robot.leftBack.setPower(speed);
            robot.rightBack.setPower(-speed);
            while(opmode.opModeIsActive()) {
                angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                //heading = AngleUnit.DEGREES.normalize(angles.firstAngle);
                heading = angles.firstAngle;
                //if(Math.abs(heading) < Math.abs(degrees)) {
                opmode.telemetry.addData("heading", heading);
                opmode.telemetry.update();
                if((heading < (degrees + 7)) && (heading > (degrees - 7))) {
                    opmode.telemetry.addLine("if statements runs");
                    opmode.telemetry.update();
                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);
                    global_angle += degrees;
                    robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    return;
                }
            }
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void driveturn(double speed, double degrees) {
        Orientation angles;
        float heading;
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        if(opmode.opModeIsActive()) {
            robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.leftFront.setPower(-0.1);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(-0.1);
            robot.rightBack.setPower(speed);
            while(opmode.opModeIsActive()) {
                angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //heading = AngleUnit.DEGREES.normalize(angles.firstAngle);
                heading = angles.firstAngle;
                if((heading < (degrees + 1)) && (heading > (degrees - 1))) {
                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);
                    return;
                }
            }
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void turn(double speed, double degrees) {
        double inches = degrees * 13.8 / 360;
        int leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget;
        if(opmode.opModeIsActive()) {
            leftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftBackTarget = robot.leftBack.getCurrentPosition() +  (int)(inches * COUNTS_PER_INCH);
            rightFrontTarget = robot.rightFront.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            rightBackTarget = robot.rightBack.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            robot.leftFront.setPower(-speed);
            robot.rightFront.setPower(speed);
            robot.leftBack.setPower(-speed);
            robot.rightBack.setPower(speed);
            if(inches >= 0) {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() <= leftFrontTarget | robot.leftBack.getCurrentPosition() <= leftBackTarget | robot.rightFront.getCurrentPosition() >= rightFrontTarget | robot.rightBack.getCurrentPosition() >= rightBackTarget)) {

                }
            } else {
                while(opmode.opModeIsActive() && (robot.leftFront.getCurrentPosition() >= leftFrontTarget | robot.leftBack.getCurrentPosition() >= leftBackTarget | robot.rightFront.getCurrentPosition() <= rightFrontTarget | robot.rightBack.getCurrentPosition() <= rightBackTarget)) {
                    robot.leftFront.setPower(0);
                    robot.rightFront.setPower(0);
                    robot.leftBack.setPower(0);
                    robot.rightBack.setPower(0);
                }
            }
        }
    }

    public int identifyRing() {
        boolean bottomRing = false;
        boolean topRing = false;
        if (robot.colorBottom.red()>300 && robot.colorBottom.blue()>177 && robot.colorBottom.green()>342) {
            bottomRing = true;
            if (robot.colorTop.red()>170 && robot.colorTop.blue()>77 && robot.colorTop.green()>183) {
                topRing = true;
            }
        }
        opmode.telemetry.addData("color 1 red", robot.colorTop.red());
        opmode.telemetry.addData("color 1 blue", robot.colorTop.blue());
        opmode.telemetry.addData("color 1 green", robot.colorTop.green());

        opmode.telemetry.addData("color 2 red", robot.colorBottom.red());
        opmode.telemetry.addData("color 2 blue", robot.colorBottom.blue());
        opmode.telemetry.addData("color 2 green", robot.colorBottom.green());
        if (bottomRing&&topRing) {
            opmode.telemetry.addData("Rings", "4");
            opmode.telemetry.update();
            return 4;
        } else if (bottomRing&&!topRing) {
            opmode.telemetry.addData("Rings", "1");
            opmode.telemetry.update();
            return 1;
        } else {
            opmode.telemetry.addData("Rings", "0");
            opmode.telemetry.update();
            return 0;
        }
    }

    public void wArmDown() {
        robot.wobbleArm.setPosition(0.2);
        opmode.sleep(200);
    }

    public void wArmUp() {
        robot.wobbleArm.setPosition(0.5);
    }

    public void wClawOpen() {
        robot.wobbleClaw.setPosition(1);
        opmode.sleep(100);
    }

    public void wClawClose() {
        robot.wobbleClaw.setPosition(0.3);
        opmode.sleep(100);
    }

    public void shooterOn() {
        robot.shooter.setPower(0.58);
    }

    public void shooterOff() {
        robot.shooter.setPower(0);
    }

    public void intakeForward() {
        robot.intake.setPower(1);
    }

    public void intakeBackward() {
        robot.intake.setPower(-1);
    }

    public void intakeOff() {
        robot.intake.setPower(0);
    }

    public void colorLightOn(){
        robot.colorBottom.enableLed(true);
        robot.colorTop.enableLed(true);
    }

    public void colorLightOff(){
        robot.colorBottom.enableLed(false);
        robot.colorTop.enableLed(false);
    }

    private ElapsedTime magTimer = new ElapsedTime();
    public void magazinePush() {
        robot.mag.setPosition(0.2);
        magTimer.reset();
        if (magTimer.seconds() > 0.75) {
            robot.mag.setPosition(0.4);
        }
    }

    public void robotWait(int ms) {
        opmode.sleep(ms);
    }

    public void parseMoves(Path[] paths) {
        for (Path path : paths) {
            if (path.move == M.DRIVE) {
                drive(path.speed, path.arg);
            } else if (path.move == M.STRAFE) {
                strafe(path.speed, path.arg);
            } else if (path.move == M.ROTATE) {
                turn(path.speed, path.arg);
            } else if (path.move == M.WOBBLE_UP) {
                wArmUp();
            } else if (path.move == M.WOBBLE_DOWN) {
                wArmDown();
            } else if (path.move == M.WOBBLE_OPEN) {
                wClawOpen();
            } else if (path.move == M.WOBBLE_CLOSE) {
                wClawClose();
            } else if (path.move == M.MAG_PUSH) {
                magazinePush();
            } else if (path.move == M.WAIT) {
                robotWait((int) path.arg);
            } else if (path.move == M.SHOOTER_ON) {
                shooterOn();
            } else if (path.move == M.SHOOTER_OFF) {
                shooterOff();
            } else if (path.move == M.INTAKE_FORWARD) {
                intakeForward();
            } else if (path.move == M.INTAKE_BACKWARD) {
                intakeBackward();
            } else if (path.move == M.INTAKE_OFF) {
                intakeOff();
            }
            /*else if (path.move == M.STRAFE_TILL) {
                strafeTillLimit(path.speed, path.arg);
            } else if (path.move == M.DRIVETURN) {
                driveturn(path.speed, path.arg);
            } else if (path.move == M.CORRECT) {
                correct();
            } */
            opmode.sleep(200);
        }
    }
}