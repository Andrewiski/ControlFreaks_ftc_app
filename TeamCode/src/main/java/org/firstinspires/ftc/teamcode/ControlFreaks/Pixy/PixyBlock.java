package org.firstinspires.ftc.teamcode.ControlFreaks.Pixy;

import java.util.Comparator;

/**
 * Created by adevries on 12/6/2017.
 */

public class PixyBlock {
    public PixyBlock() {

    }

    // print block structure!
    public String print() {
        try {
            int i, j;
            String sig = "";
            int d;
            Boolean flag;
            if (signature > 7) // color code! (CC)
            {
                // convert signature number to an octal string
                for (i = 12, j = 0, flag = false; i >= 0; i -= 3) {
                    d = (signature >> i) & 0x07;
                    if (d > 0 && !flag)
                        flag = true;
                    if (flag)
                        sig = sig + d + '0';
                }

                return "CC block! sig: " + signature + " x:" + x + " y:" + y + " width:" + width + " height:" + height + " angle:" + angle;
            } else // regular block.  Note, angle is always zero, so no need to print
                return "sig: " + signature + " x:" + x + " y:" + y + " width:" + width + "height:" + height;
        } catch (Exception ex) {

            return "Error Block.print() " + ex.getMessage();
        }

    }

    public int signature = -1;
    public int numBlocks = 1;
    public int x = -1;
    public int y = -1;
    public int width = -1;
    public int height = -1;
    public int angle = -1;

    public static int BlockYXComparatorYFudge = 10;
    public static Comparator<PixyBlock> BlockYXComparator
            = new Comparator<PixyBlock>() {


        public int compare(PixyBlock a, PixyBlock b) {

            if (Math.abs(a.y - b.y) < BlockYXComparatorYFudge) {
                //if the difference in the y < yFudge assume same column
                if (a.x > b.y)
                    return 1;
                else if (a.x < b.x)
                    return -1;
                else
                    return 0;
            } else if (a.y > b.y)
                return 1;
            else if (a.y < b.y)
                return -1;
            else
                return 0;

        }

    };

}
