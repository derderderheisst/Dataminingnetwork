/*
 * helpers.h
 *
 *  Created on: Feb 7, 2018
 *      Author: lkn
 */

#ifndef HELPERS_H_
#define HELPERS_H_

// C program for implementation of ftoa()
#include<stdio.h>
#include<math.h>


/*
 * debug prints*/
#define PRINT_TO_CONSOLE 0
#if(PRINT_TO_CONSOLE==1)

#define DEBUG_PRINTF(x...) printf(x)
#else
#define DEBUG_PRINTF(x...)
#endif

int MyPow(int a,int b){
  if(b<0)
	 return (1.0/a)*(MyPow(a,b+1));
  else if(b==0)
    return 1;
  else if(b==1)
    return a;
  else
    return a*MyPow(a,b-1);
}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * MyPow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

#endif /* HELPERS_H_ */

