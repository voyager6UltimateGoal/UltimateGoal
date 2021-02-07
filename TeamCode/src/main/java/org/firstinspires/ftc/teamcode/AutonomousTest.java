package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Drive Forward")
public class AutonomousTest extends LinearOpMode {
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
        Path[] paths = {
                //new Path(M.DRIVE, D.FORWARD, 0.5, 20),
                //new Path(M.STRAFE, D.BACKWARD, 0.5, 10),
                new Path(M.ROTATE, D.FORWARD, 0.2, 90)
        };
        driver.parseMoves(paths);
    }
}