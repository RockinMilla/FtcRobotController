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
            telemetry.addData("This code was last updated", "1/10/2026, 2:56 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        r.getPinpointPosition();

        r.intakePower(0.5);
        r.launcherVelocity(1200); // Change this when changing launcher velocity

        shootBalls(r); // Shoot preloaded balls

        r.driveToPos(500, 200, -70); // park
        r.launcherVelocity(0);
        r.intakePower(0);
    }

    private void shootBalls(RockinBot r)
    {
        r.intakePower(0.2);
        r.driveToPos(0, 100, -20, 10, 1.5 ,5);
        r.waitForLaunchers(1200); // Change this when changing launcher velocity
        r.intakePower(0.5);
        r.turnLifterToDegrees(360); // Shoot the balls
        r.waitForLifter();
        r.turnLifterToDegrees(-190); // Reset lifter
        sleep(1000);
    }
}