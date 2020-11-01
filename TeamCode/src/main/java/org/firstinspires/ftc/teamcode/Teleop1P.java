package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@TeleOp(name="Teleop 1P", group="Linear Opmode")
public class Teleop1P extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private double leftFrontPower;
    private double leftBackPower;
    private double rightFrontPower;
    private double rightBackPower;
    private double drive; //testing
    private double strafe;
    private double rotate;
    private boolean button_a;
    private boolean button_b;
    private boolean button_x;
    private boolean button_y;
    private boolean button_du;
    private boolean button_dd;
    private boolean bumper_left;
    private boolean bumper_right;
    private boolean button_dl;
    private boolean button_dr;
    // private boolean targetVisible = false;
    // private OpenGLMatrix lastLocation = null;

    private static final double INCREMENT = 0.03;
    private static final int CYCLE_MS = 50;

    private static final double wAIncrement = 0.05;
    private static final double wAMaxPos = 0.5;
    private static final double wAMinPos = 0;
    private double wAPosition = 0;
    private static final double wCIncrement = 0.05;
    private static final double wCMaxPos = 0.5;
    private static final double wCMinPos = 0;
    private double wCPosition = 0;

    private boolean shooterActive = false;
    
    // testing pull
    // testing update project

    /* limits for servos and motors
    private static final double MAX_POS = 1.0;
    private static final double MIN_POS = 0.0;
    private double position = (MAX_POS - MIN_POS) / 2;

    private static final double BINCREMENT = 0.05;
    private static final double BBMAX_POS = 0.65;
    private static final double BBMIN_POS = 0.0;
    private static final double BMAX_POS = 1;
    private static final double BMIN_POS = 0.35;
    private double bposition = BMAX_POS;
    private double bbposition = BBMIN_POS;

    private static final double CINCREMENT = 0.06;
    private static final double CMAX_POS = 1.0;
    private static final double CMIN_POS = 0.0;
    private double cposition = (0.4);

    private static final double DINCREMENT = 0.03;
    private static final double DMAX_POS = 1.0;
    private static final double DMIN_POS = 0.0;
    private double dposition = (DMAX_POS - DMIN_POS) / 2;
    private double frontIsSkystone;
    private double backIsSkystone;
    int front_red, front_green, front_blue, back_red, back_green, back_blue;
 */

    VoyagerBot robot = new VoyagerBot();
    //Detecting detector = null;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        //detector = new Detecting(this, robot);
        robot.resetEncoders();
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            drive = -gamepad1.left_stick_y; //forward and backward
            strafe = 0.90 * gamepad1.right_stick_x; // side to side and diagonal
            rotate = 0.85 * gamepad1.left_stick_x; // rotate in place
            button_a = gamepad1.a;
            button_b = gamepad1.b;
            button_x = gamepad1.x;
            button_y = gamepad1.y;
            button_du = gamepad1.dpad_up;
            button_dd = gamepad1.dpad_down;
            button_dl = gamepad1.dpad_left;
            button_dr = gamepad1.dpad_right;
            bumper_left = gamepad1.left_bumper;
            bumper_right = gamepad1.right_bumper;
            if(gamepad1.right_stick_button) {
                rotate = 0.5 * rotate;
                drive = 0.5 * drive;
                strafe = 0.5 * strafe;
            }

            // testing wobble arm and wobble claw
            if (button_a) {
                wAPosition += wAIncrement;
                if (wAPosition > wAMaxPos) {
                    wAPosition = wAMaxPos;
                }
            } if (button_b) {
                wAPosition -= wAIncrement;
                if (wAPosition < wAMinPos) {
                    wAPosition = wAMaxPos;
                }
            }
            if (button_x) {
                wCPosition += wCIncrement;
                if (wCPosition > wCMaxPos) {
                    wCPosition = wCMaxPos;
                }
            } if (button_y) {
                wCPosition -= wCIncrement;
                if (wCPosition < wCMinPos) {
                    wCPosition = wCMaxPos;
                }
            }


            if(bumper_left) {
                robot.intake.setPower(1.0);
            }
            if(bumper_right) {
                robot.intake.setPower(-1);
            }

            if(!bumper_left && !bumper_right) {
                robot.intake.setPower(0);
            }

            if(!shooterActive && button_dd) {
                robot.shooter.setPower(1);
                shooterActive = true;
            }
            if(shooterActive && button_dd) {
                robot.shooter.setPower(0);
                shooterActive = false;
            }
            /*

            if(button_a) {
                position += INCREMENT;
                if(position >= MAX_POS) {
                    position = MAX_POS;
                }
            } else if(button_b) {
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                }
            }
            if(button_dr) {
                cposition += CINCREMENT;
                if(cposition >= CMAX_POS) {
                    cposition = CMAX_POS;
                }
            } else if(button_dl) {
                cposition -= CINCREMENT;
                if (cposition <= CMIN_POS) {
                    cposition = CMIN_POS;
                }
            }
            if(button_dd) {
                bposition += BINCREMENT;
                bbposition -= BINCREMENT;
                if(bposition >= BMAX_POS) {
                    bposition = BMAX_POS;
                }
                if (bbposition <= BBMIN_POS) {
                    bbposition = BBMIN_POS;
                }
            } else if(button_du) {
                bposition -= BINCREMENT;
                bbposition += BINCREMENT;
                if (bposition <= BMIN_POS) {
                    bposition = BMIN_POS;
                }
                if (bbposition >= BBMAX_POS) {
                    bbposition = BBMAX_POS;
                }
            }
            if(gamepad2.dpad_down) {
                robot.extension.setPower(0.3);
            }
            if(gamepad2.dpad_up) {
                robot.extension.setPower(-0.3);
            }
            if(!gamepad2.dpad_up && !gamepad2.dpad_down) {
                robot.extension.setPower(0);
            }
            /*if(gamepad1.y) {
                robot.yeeter.setPower(0.7);
            } else {
                robot.yeeter.setPower(0);
            }
            if(gamepad1.x) {
                robot.yeeter.setPower(-0.7);
            } else {
                robot.yeeter.setPower(0);
            }*/ // TODO: yeeter is gone!!!!!!!!!!

            /*
            if(gamepad1.x) {
                dposition += DINCREMENT;
                if(dposition >= DMAX_POS) {
                    dposition = DMAX_POS;
                }
            } else if(gamepad1.y) {
                dposition -= DINCREMENT;
                if (dposition <= DMIN_POS) {
                    dposition = DMIN_POS;
                }
            }
            */

            /*if (((VuforiaTrackableDefaultListener)detector.stoneTarget.getListener()).isVisible()) {
                telemetry.addData("Visible Target", detector.stoneTarget.getName());
                targetVisible = true;
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)detector.stoneTarget.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
            if(targetVisible) {
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / detector.mmPerInch, translation.get(1) / detector.mmPerInch, translation.get(2) / detector.mmPerInch);
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            } else {
                telemetry.addData("Visible Target", "none");
            }*/

            /*
            front_red = robot.color_front.red();
            front_green = robot.color_front.green();
            front_blue = robot.color_front.blue();
            back_red = robot.color_back.red();
            back_green = robot.color_back.green();
            back_blue = robot.color_back.blue();
            frontIsSkystone = (front_red + front_green) / front_blue;
            backIsSkystone = (back_red + back_green) / back_blue;
            telemetry.addData("Front Skystone", frontIsSkystone);
            telemetry.addData("Back Skystone", backIsSkystone);
            telemetry.update();
             */

            /* * * * * * * * * * * *
             * Left stick:
             * - up and down moves forwards and backwards
             * - left and right rotates the robot in place
             * Right stick:
             * - strafing side to side
             * apparently you can face forward while moving in a circle
             * by strafing and rotating at the same time
             */
            leftFrontPower = drive + strafe + rotate;
            leftBackPower = drive - strafe + rotate;
            rightFrontPower = drive - strafe - rotate;
            rightBackPower = drive + strafe - rotate;

            robot.leftFront.setPower(leftFrontPower);
            robot.leftBack.setPower(leftBackPower);
            robot.rightFront.setPower(rightFrontPower);
            robot.rightBack.setPower(rightBackPower);

            robot.wobbleArm.setPosition(wAPosition);
            robot.wobbleClaw.setPosition(wCPosition);

            /*
            robot.back.setPosition(bposition);
            robot.back2.setPosition(bbposition);
            robot.claw.setPosition(position);
            robot.skystone.setPosition(cposition);
            robot.gripper.setPosition(dposition);
            */
            sleep(CYCLE_MS);
        }
    }
}
