/*
 * Copyright (c) FTC Team 15167 Robo Troopers (http://robotroopers.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.example.neil.lib;


import java.util.Scanner;

//@TODO: FINISH SINE AND COS FOR FINDING AN ANGLE

public class LawOfSineCos
{
    public static void main(String[] args)
    {

        Scanner s = new Scanner(System.in);

        System.out.println("Enter 'sin' for law of sines, 'cos' for law of cosine");
        String input = s.nextLine();

        switch(input)
        {
            case "sin":
                System.out.println("Are you looking for an angle or a side?");
                System.out.println("Enter 'a' for angle or 's' for side");
                String Question = s.nextLine();

                if(Question.equals("s"))
                {
                    double side,oppositeAngle,adg1,adg2;
                    System.out.println("Enter the value of the known side, opposite angle, and one of the adjacent angles");
                    System.out.println("The adjacent angle must be opposite to the side you want to find");
                    System.out.println("Side: ");
                    side = s.nextDouble();
                    System.out.println("Opposite Angle: ");
                    oppositeAngle = s.nextDouble();
                    System.out.println("Other known angle: ");
                    adg1 = s.nextDouble();
                    adg2 = 180 - oppositeAngle - adg1;

                    System.out.println("Is the other angle: "+adg2+"?");
                    System.out.println("If it is not, type 'no'. If it is, type 'yes'");
                    String no = s.nextLine();

                    if(no.equals("no"))
                    {
                        System.out.println("Enter the other angle: ");
                        adg2 = s.nextDouble();
                    }

                    else {
                        System.out.println();
                    }

                    double oppositeSin = Math.sin(Math.toRadians(oppositeAngle)); // switching the angle given into radians
                    double adjSin = Math.sin(Math.toRadians(adg1));               // same thing here
                    double otherAdjSin = Math.sin(Math.toRadians(adg2));

                    double endRes = (side/oppositeSin)*adjSin;                    // endres is the unknown side
                    double otherSide = (side/oppositeSin)*otherAdjSin;            // otherSide is the other unknown side

                    System.out.println("The first side that you wanted to find is "+endRes+" The other side length is "+otherSide);


                }

                break;

            case "cos":

                System.out.println("Do you want to find out a side or an angle? Type 'side' for side and 'angle' for angle");

                String input1 = s.nextLine();

                if(input1.equals("side"))
                {
                    double a, b, C;

                    System.out.println("The law of cosines is: c^2 = a^2 + b^2 - 2 a b cos C");
                    System.out.println("C must be the opposite angle of line c");
                    System.out.println("Enter line a length ");
                    a = s.nextDouble();
                    System.out.println("Enter line b length ");
                    b = s.nextDouble();
                    System.out.println("Enter angle C ");
                    C = s.nextDouble();


                    double angle =  C;
                    double result  = Math.cos( Math.toRadians( angle ) ) ;
                    System.out.println("The equation is: c^2 = "+a*a+" + "+b*b+" - "+2*a*b*result);
                    double right = a*a + b*b - 2*a*b*result;
                    double left = Math.sqrt(right);
                    System.out.println("Line c is equal to "+left);
                }
        }
    }
}