package org.firstinspires.ftc.teamcode.ControlFreaks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by adevries on 10/7/2017.
 */
@TeleOp(name="Encoder Tests", group="Manual" )  // @Autonomous(...) is the other common choice
//@Disabled
public class CFPushBotEncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    CFPushBotHardware robot;   // Use a Pushbot's hardware


    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            /* Declare OpMode members. */
            robot           = new CFPushBotHardware();   // Use a Pushbot's hardware

            robot.init(this);
            robot.debugOn();
            //robot.sensor_range_init();

            //robot.sensor_color_enable(true);
            //robot.sensor_color_led(true);
            //robot.sensor_range_enable(true);
            //robot.sensor_pixy_init();
            //robot.sensor_pixy_maxsignature_enable(0,true);
            //Enable Pixy witch will start the i2c queries
            //robot.sensor_pixy_enable(true);
            // Wait for the game to start (driver presses PLAY)
            //robot.led7seg_timer_init(600);
            waitForStart();
            //robot.led7seg_timer_start(600);
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();
                    //String msg = "";
                    //msg = msg + "range: " + robot.sensor_range_get_distance() + "\n";
                    //msg = msg + "Color " +  robot.sensor_color_GreatestColor() + "\n";
                    //int blockcolor = -1;
                    //int tempBlockcolor = robot.sensor_pixy_getjewelcolor(true);
                    //if (tempBlockcolor > 0){
                    //    blockcolor = tempBlockcolor;
                    //}
                    //robot.set_message(msg);
                    sleep(10);
                }catch(Exception ex){
                    robot.set_error_message("Fatal Error "  + ex.toString());
                }
            } // loop

        }catch(Exception ex){
            robot.set_error_message("Fatal Error "  + ex.toString());
        }
    }


}
