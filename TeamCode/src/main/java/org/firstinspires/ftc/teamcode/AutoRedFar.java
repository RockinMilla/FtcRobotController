package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Far", group="Robot")
public class AutoRedFar extends LinearOpMode {
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");     // Passing in code from RockinBot

        telemetry.addData("Autonomous Ready", "You can press start now");
        telemetry.addData("This code was last updated", "1/26/2026, 12:53 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();

        r.intakePower(0.5);
        r.launcherVelocity(1200); // Change this when changing launcher velocity

        shootBalls(r); // Shoot preloaded balls

        r.driveToPos(400, 200, -45); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r) {
        r.intakePower(0.2);
        r.driveToPos(0, 100, -20, 10, 1.5, 5);
        r.waitForLaunchers(1200); // Change this when changing launcher velocity
        r.intakePower(0.5);
        r.turnLifterToDegrees(90); // Shoot ball 1
        sleep(500);
        r.turnLifterToDegrees(180); // Shoot ball 2
        sleep(500);
        r.turnLifterToDegrees(270); // Shoot ball 3
        r.waitForLifter();
        sleep(1000);
    }
}