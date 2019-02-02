package com.example.neil.lib;

import java.util.Scanner;


public class RootSimplifier
{
    public static void main(String[] args)
    {
        int x;

        Scanner s = new Scanner(System.in);

        System.out.println("enter a:");

        x = s.nextInt();

        for(int a = 1; a <= x; a++)
        {
            if(x % a == 0)
            {
                System.out.println(a + " ");
            }
        }
    }
}