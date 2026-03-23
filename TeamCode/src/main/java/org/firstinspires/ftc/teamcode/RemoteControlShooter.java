package org.firstinspires.ftc.teamcode;

// All the things that we use and borrow
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Remote Control Shooter", group="Linear OpMode")
public class RemoteControlShooter extends LinearOpMode {
    private DigitalChannel laserInput;
    @Override

    //Op mode runs when the robot runs. It runs the whole time.
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Create a LinearOpModeVariable and pass it to the RockinBot constructor
        LinearOpMode o = this;
        RockinBot r = new RockinBot(o, "Shooter");

        // THESE ARE THE VARIABLES THAT ARE RUNNING DURING RC, NOT THE ONES IN ROCKINBOT!!
        // These are the defaults that run when the program starts. Their values can be modified by RC inputs
        double launcherSpeed = 850; // figure this out
        double closeLauncherSpeed = 850;
        double mediumLauncherSpeed = 920;
        double longLauncherSpeed = 1140;
        double intakeSpeed = 1;
        double lifterPower = 0;
        boolean park = false;
        boolean bumperPressed = false;
        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Remote Control Ready", "press PLAY");
        RobotLog.vv("Rockin' Robots", "Remote Control Ready");
        telemetry.addData("This code was last updated", "1/8/2026, 4:35 pm"); // Todo: Update this date when the code is updated
        telemetry.update();

        waitForStart();
        r.intakePower(intakeSpeed);
        r.launcherVelocity(launcherSpeed);
        r.lifterPower(lifterPower);

        // Read the sensor state (true = HIGH, false = LOW)
        boolean stateHigh = laserInput.getState();

        // Active-HIGH: HIGH means an object is detected
        boolean detected = stateHigh;

        // Display detection state
        if (detected) {
            r.
        } else {
            telemetry.addLine("No object detected");
        }


        TelemetryPacket packetField = new TelemetryPacket();

        packetField.fieldOverlay()
                .drawImage("/Downloads/field.png", 20, 0, 144, 144);
        dashboard.sendTelemetryPacket(packetField);

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //The centering of the robot is wrong for now
            TelemetryPacket packetRobot = new TelemetryPacket();
            r.getPinpointPosition();
            packetRobot.fieldOverlay()
                    .setTranslation(5, 5)
                    .setRotation(r.getHLoc())
                    .setFill("yellow")
                    .fillRect(r.getXloc(), r.getYloc(), 20, 20);

            dashboard.sendTelemetryPacket(packetRobot);

            if (gamepad1.dpadRightWasReleased() && launcherSpeed == closeLauncherSpeed) {
                launcherSpeed = mediumLauncherSpeed;
                r.launcherVelocity(mediumLauncherSpeed);
            }
            if(gamepad1.dpad_right && launcherSpeed == mediumLauncherSpeed) {
                launcherSpeed = longLauncherSpeed;
                r.launcherVelocity(longLauncherSpeed);
            }

            if (gamepad1.dpadLeftWasReleased() && launcherSpeed == longLauncherSpeed) {
                launcherSpeed = mediumLauncherSpeed;
                r.launcherVelocity(mediumLauncherSpeed);
            }
            if (gamepad1.dpad_left && launcherSpeed == mediumLauncherSpeed) {
                launcherSpeed = closeLauncherSpeed;
                r.launcherVelocity(closeLauncherSpeed);
            }

            if(gamepad1.circle){
                intakeSpeed = 1;
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.square){
                intakeSpeed = -1;
                r.intakePower(intakeSpeed);
            }
            else if(gamepad1.cross){
                intakeSpeed = 0;
                r.intakePower(intakeSpeed);
            }

            if(gamepad1.right_trigger > 0){
                lifterPower = 1500;
                r.lifterPower(lifterPower);
            }

            if (gamepad1.left_trigger > 0){
                lifterPower = -1500;
                r.lifterPower(lifterPower);
            }

            if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0){
                lifterPower = 0;
                r.lifterPower(lifterPower);
            }

            if(gamepad1.dpad_down){
                park = true;
            }
            else if(gamepad1.dpad_up){
                park = false;
            }
            r.setWheelPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, park);

            /////////////////////////////////////////////////////////////
            // This is our test code.
            if(r.touchSensor.isPressed()){
                gamepad2.rumble(1000);
                gamepad2.setLedColor(255, 255, 0, 1000);
            }
            else{
                gamepad2.stopRumble();
            }

            r.getColor();

            if(gamepad2.dpad_down){
                r.setpValue(-0.1);
            }
            if(gamepad2.dpad_up){
                r.setpValue(0.1);
            }
            // This code controls our LED light using the right trigger.
            r.led.setPosition(gamepad2.right_trigger);

            // Show the elapsed game time and wheel power.
            r.printDataOnScreen();
        }
    }
}