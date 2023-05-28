package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;




@TeleOp(name="Driver OpMode", group="jabernaut1")
public class DriverOpMode extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private long loops = 0L;

    private List<LynxModule> hubModules = null;

    private DcMotorEx wheelFrontLeft = null;
    private DcMotorEx wheelFrontRight = null;
    private DcMotorEx wheelBackLeft = null;
    private DcMotorEx wheelBackRight = null;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        final String CAPTION_STATUS = "<b>Status</b>";
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setCaptionValueSeparator(": ");
        telemetry.addData(CAPTION_STATUS, "<i>Initializing…</i>");
        telemetry.update();


        // Query control hub data in bulk, but require manual cache flushing
        hubModules = hardwareMap.getAll(LynxModule.class);
        for (final LynxModule hubModule : hubModules) {
            hubModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        // Configure drive wheels
        final DcMotorEx.Direction directionLeft = DcMotorSimple.Direction.FORWARD;
        final DcMotorEx.Direction directionRight = DcMotorSimple.Direction.REVERSE;

        wheelFrontLeft = hardwareMap.get(DcMotorEx.class, "WheelFrontLeft");
        wheelFrontLeft.setDirection(directionLeft);

        wheelFrontRight = hardwareMap.get(DcMotorEx.class, "WheelFrontRight");
        wheelFrontRight.setDirection(directionRight);

        wheelBackLeft = hardwareMap.get(DcMotorEx.class, "WheelBackLeft");
        wheelBackLeft.setDirection(directionLeft);

        wheelBackRight = hardwareMap.get(DcMotorEx.class, "WheelBackRight");
        wheelBackRight.setDirection(directionRight);


        // Signal initialization complete
        telemetry.addData(CAPTION_STATUS, "<i style=\"color: green\">Ready.</i>");
        gamepad1.rumbleBlips(1);
        gamepad2.rumbleBlips(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }




    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        loops = 0L;

        telemetry.setMsTransmissionInterval(100);

        // Signal that opmode started
        gamepad1.rumbleBlips(1);
        gamepad2.rumbleBlips(1);
    }

    private static final double DEAD_ZONE_RADIUS = 0.1;
    private static final double STICK_LINEAR_TO_MOTORPOWER_POWER = 3.0;
    private static Vec2D getGamepadStickVector(final float x, final float y)
    {
        final Vec2D stick = new Vec2D(x, -y);

        // Normalize and apply dead-zone
        final double liveZoneMagnitude = Range.scale(stick.getMagnitude(),
            DEAD_ZONE_RADIUS, 1.0,  // From range
            0.0, 1.0);  // To range
        stick.changeMagnitude(liveZoneMagnitude);

        return stick;
    }
    private static Vec2D getGamepadStickLeftVector(final Gamepad gamepad)
    {
        return getGamepadStickVector(gamepad.left_stick_x, gamepad.left_stick_y);
    }
    private static Vec2D getGamepadStickRightVector(final Gamepad gamepad)
    {
        return getGamepadStickVector(gamepad.right_stick_x, gamepad.right_stick_y);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("<b>Rate</b>", "<tt>%.1f</tt> Hz", ++loops / runtime.seconds());


        // Flush hub sensor caches
        for (final LynxModule hubModule : hubModules) {
            hubModule.clearBulkCache();
        }


        // Sample inputs
        final Vec2D gamepad1StickLeft = getGamepadStickLeftVector(gamepad1);
        final Vec2D gamepad1StickRight = getGamepadStickRightVector(gamepad1);

        final Vec2D gamepad2StickLeft = getGamepadStickLeftVector(gamepad2);
        final Vec2D gamepad2StickRight = getGamepadStickRightVector(gamepad2);


        final Vec2D driveTranslation = gamepad1StickLeft;
        double driveRotation = gamepad1StickRight.getX();
        telemetry.addData("<b>Drive Translation</b>", "<i>X</i>=<tt>%3.0f</tt>%%, <i>Y</i>=<tt>%3.0f</tt>%%",
            driveTranslation.getX() * 100.0, driveTranslation.getY() * 100.0);
        telemetry.addData("<b>Drive Rotation</b>", "<tt>%3.0f</tt>%%",
            driveRotation * 100.0);


        // Apply drive wheel powers
        {
            // Convert linear stick percent to power for approximately linear velocity
            driveTranslation.changeMagnitude(
                Math.pow(driveTranslation.getMagnitude(), STICK_LINEAR_TO_MOTORPOWER_POWER));
            driveRotation = Math.pow(driveRotation, STICK_LINEAR_TO_MOTORPOWER_POWER);

            // Combine for mechanum wheels
            double powerFrontLeft  = driveTranslation.getY() + driveTranslation.getX() + driveRotation;
            double powerFrontRight = driveTranslation.getY() - driveTranslation.getX() - driveRotation;
            double powerBackLeft   = driveTranslation.getY() - driveTranslation.getX() + driveRotation;
            double powerBackRight  = driveTranslation.getY() + driveTranslation.getX() - driveRotation;

            // Normalize to a max of 100% power
            final double powerMax = Math.max(
                Math.max(Math.abs(powerFrontLeft), Math.abs(powerFrontRight)),
                Math.max(Math.abs(powerBackLeft), Math.abs(powerBackRight)));
            if (powerMax > 1.0) {
                powerFrontLeft /= powerMax;
                powerFrontRight /= powerMax;
                powerBackLeft /= powerMax;
                powerBackRight /= powerMax;
            }

            // TODO: Use velocity-mode PID/PIDF.
            wheelFrontLeft.setPower(powerFrontLeft);
            wheelFrontRight.setPower(powerFrontRight);
            wheelBackLeft.setPower(powerBackLeft);
            wheelBackRight.setPower(powerBackRight);

            telemetry.addData("<b>Front</b>", "<i>Left</i>=<tt>%3.0f</tt>%%, <i>Right</i>=<tt>%3.0f</tt>%%",
                powerFrontLeft * 100.0, powerFrontRight * 100.0);
            telemetry.addData("<b>Back</b>", "<i>Left</i>=<tt>%3.0f</tt>%%, <i>Right</i>=<tt>%3.0f</tt>%%",
                powerBackLeft * 100.0, powerBackRight * 100.0);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        wheelFrontLeft.setPower(0.0);
        wheelFrontRight.setPower(0.0);
        wheelBackLeft.setPower(0.0);
        wheelBackRight.setPower(0.0);
    }
}
