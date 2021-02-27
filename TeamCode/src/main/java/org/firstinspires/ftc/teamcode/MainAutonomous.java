package org.firstinspires.ftc.teamcode;

import android.graphics.MaskFilter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Main Autonomous")
public class MainAutonomous extends LinearOpMode {
    Driving driver = new Driving(this);
    @Override
    public void runOpMode() {
        driver.initHwMap();
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        driver.resetEncoders();
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();
        driver.colorLightOn();
        Path[] initial = { // drive the robot into position to scan the rings
                new Path(M.STRAFE, D.BACKWARD, 0.11, 38),
        };
        driver.colorLightOff();
        driver.parseMoves(initial);
        Path[] noRing = { // what the robot does if there's no rings
                new Path(M.STRAFE, D.BACKWARD, 0.15, 40),
                new Path(M.DRIVE, D.BACKWARD, 0.1, 5),
                new Path(M.WOBBLE_DOWN, D.N, 0, 0),
                new Path(M.WOBBLE_OPEN, D.N, 0, 0),
                new Path(M.WOBBLE_UP, D.N, 0, 0),
                new Path(M.SHOOTER_ON, D.N, 0.55, 0),
                new Path(M.STRAFE, D.FORWARD, 0.2, 18),
                new Path(M.ROTATE, D.BACKWARD, 0.2, 77),
                new Path(M.MAG_PUSH, D.N, 0, 20),
                new Path(M.SHOOTER_ON, D.N, 0.52, 0),
                new Path(M.ROTATE, D.BACKWARD, 0.1, 5),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.ROTATE, D.BACKWARD, 0.1, 6),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.SHOOTER_OFF, D.N, 0, 0),
                new Path(M.DRIVE, D.BACKWARD, 0.1, 12)
        };
        Path[] oneRing = { // what the robot does if there's one ring
                new Path(M.STRAFE, D.BACKWARD, 0.15, 56),
                new Path(M.DRIVE, D.BACKWARD, 0.15, 30),
                new Path(M.WOBBLE_DOWN, D.N, 0, 0),
                new Path(M.WOBBLE_OPEN, D.N, 0, 0),
                new Path(M.WOBBLE_UP, D.N, 0, 0),
                new Path(M.STRAFE, D.FORWARD, 0.2, 34),
                new Path(M.SHOOTER_ON, D.N, 0.55, 0),
                new Path(M.DRIVE, D.FORWARD, 0.15, 25),
                new Path(M.ROTATE, D.BACKWARD, 0.2, 79),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.ROTATE, D.BACKWARD, 0.1, 4),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.ROTATE, D.BACKWARD, 0.1, 6),
                new Path(M.SHOOTER_ON, D.N, 0.52, 0),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.SHOOTER_OFF, D.N, 0, 0),
                new Path(M.DRIVE, D.BACKWARD, 0.1, 10)
        };
        Path[] fourRing = { // what the robot does if there's four rings
                new Path(M.STRAFE, D.BACKWARD, 0.15, 80),
                new Path(M.DRIVE, D.BACKWARD, 0.1, 5),
                new Path(M.WOBBLE_DOWN, D.N, 0, 0),
                new Path(M.WOBBLE_OPEN, D.N, 0, 0),
                new Path(M.WOBBLE_UP, D.N, 0, 0),
                new Path(M.SHOOTER_ON, D.N, 0.54, 0),
                new Path(M.STRAFE, D.FORWARD, 0.2, 55),
                new Path(M.ROTATE, D.BACKWARD, 0.2, 79),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.ROTATE, D.BACKWARD, 0.1, 5),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.ROTATE, D.BACKWARD, 0.1, 5),
                new Path(M.SHOOTER_ON, D.N, 0.53, 0),
                new Path(M.MAG_PUSH, D.N, 0, 0),
                new Path(M.SHOOTER_OFF, D.N, 0, 0),
                new Path(M.DRIVE, D.BACKWARD, 0.1, 10)
        };
//        int rings = 0;
        int rings = driver.identifyRing();
        telemetry.addData("Rings", rings);
        telemetry.update();
        if (rings == 0) {
            driver.parseMoves(noRing);
        } else if (rings == 1) {
            driver.parseMoves(oneRing);
        } else {
            driver.parseMoves(fourRing);
        }
    }
}