package org.firstinspires.ftc.teamcode.ControlFreaks;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyBlockList;

/**
 * Created by adevries on 10/7/2017.
 */
@TeleOp(name="Sensor Tests", group="Manual" )  // @Autonomous(...) is the other common choice
//@Disabled
public class CFPushBotSensorTest extends LinearOpMode {

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
            robot.sensor_range_init();
            //robot.sensor_color_init(true);
            robot.sensor_color_enable(true);
            robot.sensor_color_led(true);
            robot.sensor_range_enable(true);
            robot.sensor_pixy_init();
            robot.sensor_pixy_maxsignature_enable(0,true);
            //Enable Pixy witch will start the i2c queries
            robot.sensor_pixy_enable(true);
            // Wait for the game to start (driver presses PLAY)
            robot.led7seg_timer_init(600);
            waitForStart();
            robot.led7seg_timer_start(600);
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();
                    String msg = "";
                    msg = msg + "range: " + robot.sensor_range_get_distance() + "\n";
                    msg = msg + "Color " +  robot.sensor_color_GreatestColor() + "\n";
                    PixyBlockList sigMaxBlockList_All;
                    int blockcolor = -1;
                    int Sig1X = -1;//Sig 1 is red
                    int Sig2X = -1; //Sig 2 is Blue
                    sigMaxBlockList_All = robot.sensor_pixy_maxSignatureBlocks(0);
                    msg = msg + "Pixy Blocks: " + sigMaxBlockList_All.BlockCount;
                    if(sigMaxBlockList_All.BlockCount > 0) {

                        for (int i = 0; i < sigMaxBlockList_All.BlockCount; i++) {
                            //we assume the largest blue and red are the balls so will come first
                            if (sigMaxBlockList_All.Blocks[i].signature == 1 && Sig1X == -1) {
                                Sig1X = sigMaxBlockList_All.Blocks[i].x;
                            } else if (sigMaxBlockList_All.Blocks[i].signature == 2 && Sig2X == -1) {
                                Sig2X = sigMaxBlockList_All.Blocks[i].x;
                            }
                            if (Sig1X > 0 && Sig2X > 0) {
                                break; //exit the 4 loop we already found the largest Red and Blue
                            }

                        }
                        //The Block we care abount is always on the Left so lowest X value
                        if(Sig1X > 0 && Sig2X > 0) {
                            if (Sig1X < Sig2X) {
                                blockcolor = 0;  //Ball on Left is Signature 1 Red
                            } else {
                                blockcolor = 2;  //Ball on Left is Signature 2 Blue
                            }
                        }
                        msg = msg + "pixy jewel is ";
                        if(blockcolor == 0){
                            msg= msg + "red\n";
                        }else if(blockcolor == 2){
                            msg= msg + "blue\n";
                        }else{
                            msg= msg + "n/a\n";
                        }
                        msg = msg + sigMaxBlockList_All.print() + "\n";
                    }

                    robot.set_message(msg);
                    sleep(5);
                }catch(Exception ex){
                    robot.set_error_message("Fatal Error "  + ex.toString());
                }
            } // loop

        }catch(Exception ex){
            robot.set_error_message("Fatal Error "  + ex.toString());
        }
    }


}
