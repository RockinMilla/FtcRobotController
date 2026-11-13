package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name="Remote Control", group="Linear OpMode")
public class RemoteControl extends LinearOpMode {
    private final VelocityGraph velocityGraph = VelocityGraph.getInstance();
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {

        double intakePower = 1;
        double triggerPower = 0.85;
        double leftShooterVelocity = 1725;
        double rightShooterVelocity = 1725;
        double leftPValue = 70;
        double rightPValue = 100;
        boolean halfWheelSpeed = false;
        double wheelSpeedMultiplier = 1.0;

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o);
        r.setRightPValue(rightPValue);
        r.setLeftPValue(leftPValue);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "11/5/2026, 6:11 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        waitForStart();
        r.intakePower(intakePower);
        r.shooterVelocity(leftShooterVelocity, rightShooterVelocity);
        ElapsedTime telemetryTimer = new ElapsedTime();

        velocityGraph.start();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            r.setWheelPower(
                -gamepad1.left_stick_y * wheelSpeedMultiplier,
                -gamepad1.left_stick_x * wheelSpeedMultiplier,
                gamepad1.right_stick_x * wheelSpeedMultiplier);

            // Control the trigger
            if(gamepad1.right_trigger > 0) {
                r.triggerPower(triggerPower);
            }
            else if(gamepad1.left_trigger > 0) {
                r.triggerPower(-triggerPower);
            }
            else {
                r.triggerPower(0);
            }

            // Control the intake
            if(gamepad1.circle) {
                r.intakePower(intakePower);
            }
            else if(gamepad1.square) {
                r.intakePower(-intakePower);
            }
            else if(gamepad1.cross) {
                r.intakePower(0);
            }

            if (gamepad1.dpadDownWasReleased()) {
                halfWheelSpeed = !halfWheelSpeed;
                wheelSpeedMultiplier = halfWheelSpeed ? 0.5 : 1;
            }

            // Control the lumberjack
            if (gamepad1.dpad_right) { //down
                r.setLumberjackPositions(0.1, 0.70);
            }
            if (gamepad1.dpad_left) { //up
                r.setLumberjackPositions(0.73, 0.03); //Needs to be changed once we have wheels
            }

            // Gamepad2 stuff - find good values for trigger, pValue, and intake
            if (gamepad2.dpadLeftWasReleased()) {
                rightPValue = rightPValue - 2;
                r.adjustRightPValue(-2);
            }
            if (gamepad2.dpadRightWasReleased()) {
                rightPValue = rightPValue + 2;
                r.adjustRightPValue(2);
            }
            if(gamepad2.dpadUpWasReleased())
            {
                leftPValue = leftPValue + 2;
                r.adjustLeftPValue(2);
            }
            if(gamepad2.dpadDownWasReleased())
            {
                leftPValue = leftPValue - 2;
                r.adjustLeftPValue(-2);
            }

            if (gamepad2.circleWasReleased()) {
                rightShooterVelocity = rightShooterVelocity + 5;
                r.shooterVelocity(leftShooterVelocity, rightShooterVelocity);
            }
            if (gamepad2.squareWasReleased()) {
                rightShooterVelocity = rightShooterVelocity - 5;
                r.shooterVelocity(leftShooterVelocity, rightShooterVelocity);
            }
            if (gamepad2.triangleWasReleased()) {
                leftShooterVelocity = leftShooterVelocity + 5;
                r.shooterVelocity(leftShooterVelocity, rightShooterVelocity);
            }
            if (gamepad2.xWasReleased()) {
                leftShooterVelocity = leftShooterVelocity - 5;
                r.shooterVelocity(leftShooterVelocity, rightShooterVelocity);
            }

            if (gamepad2.rightBumperWasReleased()){
                intakePower = Math.min(1, intakePower + 0.05);
                r.intakePower(intakePower);
            }
            if (gamepad2.leftBumperWasReleased()){
                intakePower = Math.max(-1, intakePower - 0.05);
                r.intakePower(intakePower);
            }

            // every loop
            
            velocityGraph.record(r.getLeftShooterVelocity(), r.getRightShooterVelocity(),
                    leftShooterVelocity, rightShooterVelocity, leftPValue, rightPValue);
            

            // Show the elapsed game time and wheel power, but only every half-second.
            if (telemetryTimer.seconds() >= 0.5) {
                telemetry.addData("Trigger power setting", "%.0f%%", triggerPower * 100);
                telemetry.addData("Intake power setting", "%.0f%%", intakePower * 100);
                telemetry.addData("Wheel speed multiplier", "%.0f%%", wheelSpeedMultiplier * 100);
                r.printDataOnScreen();
                telemetryTimer.reset();
            }
            idle();
        }
        //r.releaseLumberjacks();
    }
}