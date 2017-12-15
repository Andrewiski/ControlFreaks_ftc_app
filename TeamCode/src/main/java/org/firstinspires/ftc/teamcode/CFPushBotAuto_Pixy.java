package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;
import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyBlockList;
import org.firstinspires.ftc.teamcode.ControlFreaks.Pixy.PixyCamera;


/**
 * Created by adevries on 11/6/2015.
 */
@Autonomous(name="Pixy", group="MrD")
@Disabled
public class CFPushBotAuto_Pixy extends LinearOpMode
{
    /* Declare OpMode members. */
        CFPushBotHardware robot   = new CFPushBotHardware();
        byte red = (byte)255;
        byte green = (byte)0;
        byte blue = (byte)0;



    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during auto-operation.
     * The state machine uses a class member and encoder input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(this);
        robot.blueled_on();
        robot.setupAutoDrive();
        robot.sensor_range_init();
        robot.sensor_pixy_init();
        waitForStart();
        PixyBlockList sigMaxBlockList1_Red;
        robot.debugOn();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.hardware_loop();
            switch (v_state) {
                //
                // Synchronize the state machine and hardware.
                //
                case 0:
                    //v_state = 100;

                    v_state++;
                    break;
                case 1:

                    //
                    // Transition to the next state when this method is true
                    //if (robot.sensor_pixy_set_leds((byte)0,(byte)255,(byte)0)) {
                        //
                    //    robot.set_message("Set Led Color");

                    //}
                    v_state++;

                    break;
                case 2:
                    robot.set_message("Pixy Enabled");
                    //robot.sensor_pixy_led_external(true);
                    //enable Largest Object
                    //robot.sensor_pixy_signature_enable(0,true);
                    //Enable Signature 1

                    //robot.sensor_pixy_signature_enable(1,true);
                    //Set Color Code query to Sig 3 & 4  34 in Octal converted to Dec is 28
                    //robot.sensor_pixy_signature_colorcode_set(81);  //14 octal red & White = 12 dec
                    //Enable Color Code Querys
                    //robot.sensor_pixy_signature_enable(8,true);
                    robot.sensor_pixy_maxsignature_enable(3,true);
                    //Enable Pixy witch will start the i2c queries
                    robot.sensor_range_enable(true);
                    robot.sensor_pixy_enable(true);
                    v_state++;
                    break;
                case 3:
                    String dbg = "";
                    /*PixyCamera.Block largestBlock;
                    largestBlock = robot.sensor_pixy_signatureBlock(0);

                    if(largestBlock != null) {
                        dbg = "lb:" + largestBlock.print()+ "\n";
                    }else{
                        dbg ="lb: null\n" ;
                    }
                    PixyCamera.Block sigBlock1;
                    sigBlock1 = robot.sensor_pixy_signatureBlock(3);
                    if(sigBlock1 != null) {
                        dbg = dbg + " s3:" + sigBlock1.print() + "\n";
                    }else{
                        dbg =dbg + " s3: null\n" ;
                    }

                    PixyCamera.Block sigBlock8;
                    sigBlock8 = robot.sensor_pixy_signatureBlock(8);
                    if(sigBlock8 != null) {
                        dbg = dbg + " s8:" + sigBlock8.print() + "\n";
                    }else{
                        dbg =dbg + " s8: null\n" ;
                    }
                    */

                    //we are assuming we are pointed toward the crypto box


                    sigMaxBlockList1_Red = robot.sensor_pixy_maxSignatureBlocks(1);
                    double range = robot.sensor_range_get_distance();
                    if( range > 20) {
                        dbg = dbg + "range:" + range;
                        if (sigMaxBlockList1_Red != null) {
                            dbg = dbg + " smcc:" + sigMaxBlockList1_Red.print() + "\n";
                            if (sigMaxBlockList1_Red.BlockCount > 4) {
                                //we need to find the red blocks on near y sort by x less then 100 ie bottom half of screen
                                sigMaxBlockList1_Red.SortBottomLeftTopRight(20); //sort them on Y then X with X Fudge of 20 to handle Tilt
                                // we assume that in order to be a valid CryptoCube the distance between Blocks should be the same as the height ie square cubes
                                dbg = dbg + " sorted:" + sigMaxBlockList1_Red.print() + "\n";

                            }
                        } else {

                        }
                    }else{

                    }
                    robot.set_message(dbg);
                    //v_state++;
                    break;
                default:
                    //
                    // The autonomous actions have been accomplished (i.e. the state has
                    // transitioned into its final state.
                    //
                    break;
            }



        } // loop
    }
    //--------------------------------------------------------------------------
    //
    // v_state
    //
    /**
     * This class member remembers which state is currently active.  When the
     * start method is called, the state will be initialized (0).  When the loop
     * starts, the state will change from initialize to state_1.  When state_1
     * actions are complete, the state will change to state_2.  This implements
     * a state machine for the loop method.
     */
    private int v_state = 0;


}
