/***********************************************************************************************************************
 * Copyright [2015-2021] Renesas Electronics Corporation and/or its licensors. All Rights Reserved.
 *
 * This file is part of Renesas SynergyTM Software Package (SSP)
 *
 * The contents of this file (the "contents") are proprietary and confidential to Renesas Electronics Corporation
 * and/or its licensors ("Renesas") and subject to statutory and contractual protections.
 *
 * This file is subject to a Renesas SSP license agreement. Unless otherwise agreed in an SSP license agreement with
 * Renesas: 1) you may not use, copy, modify, distribute, display, or perform the contents; 2) you may not use any name
 * or mark of Renesas for advertising or publicity purposes or in connection with your use of the contents; 3) RENESAS
 * MAKES NO WARRANTY OR REPRESENTATIONS ABOUT THE SUITABILITY OF THE CONTENTS FOR ANY PURPOSE; THE CONTENTS ARE PROVIDED
 * "AS IS" WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, AND NON-INFRINGEMENT; AND 4) RENESAS SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING DAMAGES RESULTING FROM LOSS OF USE, DATA, OR PROJECTS, WHETHER IN AN ACTION OF
 * CONTRACT OR TORT, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THE CONTENTS. Third-party contents
 * included in this file may be subject to different terms.
 **********************************************************************************************************************/
/***********************************************************************************************************************
* File Name    : hal_entry.c
* Description  : This is a very simple example application that blinks all LEDs on a board.
***********************************************************************************************************************/

/*ROW POINTERS AND REGISTER VARIABLES*/
/*In the inner loop (l) only the column changes. The address of each row (row*width + column offset)
  is computed once per k into a pointer (cr, pr, qr), so the inner loop only does ptr[l].
  The variables of the inner loop are declared "register" so that at -O0 they are kept in CPU registers
  instead of being loaded/stored to the stack in every iteration*/

#include "hal_data.h"
#include "stdio.h"
#include "string.h"
#include "images.h"


#define N 10    /*Frame dimension for QCIF format*/
#define M 10     /*Frame dimension for QCIF format*/

#define B 5      /*Block size*/
#define Bx B     /*Block rows    (divisor of N)*/
#define By B     /*Block columns (divisor of M)*/

#define p 7       /*Search space. Restricted in a [-p,p] region around the
                    original location of the block.*/


#define s_loop(s)                                                               \
    min1 = 255*Bx*By;                                                           \
    min2 = 255*Bx*By;                                                           \
    i_loop(-s);                                                                 \
    c_loop;                                                                     \
    i_loop(s);                                                                  \
    vx += bestx;                                                                \
    vy += besty;


/*Central candidate is common between x and y axis*/
#define c_loop                                                                  \
    distc = 0;                                                                  \
    for(k=Bx-1; k>=0; k--)                                                         \
    {                                                                           \
        cr = cb[k];                                                             \
        pr = &prev_pad[bx+vx+k+p][by+vy+p];                                     \
        for(l=By-1; l>=0; l--)                                                     \
        {                                                                       \
            d = cr[l]-pr[l];                                                    \
            distc += (d<0) ? -d : d;                                            \
        }                                                                       \
    }                                                                           \
    if(distc < min1)                                                            \
    {                                                                           \
        min1 = distc;                                                           \
        bestx = 0;                                                              \
    }                                                                           \
    if(distc < min2)                                                            \
    {                                                                           \
        min2 = distc;                                                           \
        besty = 0;                                                              \
    }


/*Side candidates (i=-s, i=+s). X and Y search merged in one loop*/
#define i_loop(i)                                                               \
    distx = 0;                                                                  \
    disty = 0;                                                                  \
    for(k=Bx-1; k>=0; k--)                                                         \
    {                                                                           \
        cr = cb[k];                                                             \
        pr = &prev_pad[bx+vx+i+k+p][by+vy+p];   /*row of the X candidate*/      \
        qr = &prev_pad[bx+vx+k+p][by+vy+i+p];   /*row of the Y candidate*/      \
        for(l=By-1; l>=0; l--)                                                     \
        {                                                                       \
            p1 = cr[l];                                                         \
            d = p1-pr[l];                                                       \
            distx += (d<0) ? -d : d;                                            \
            d = p1-qr[l];                                                       \
            disty += (d<0) ? -d : d;                                            \
        }                                                                       \
    }                                                                           \
    if(distx < min1)                                                            \
    {                                                                           \
        min1 = distx;                                                           \
        bestx = i;                                                              \
    }                                                                           \
    if(disty < min2)                                                            \
    {                                                                           \
        min2 = disty;                                                           \
        besty = i;                                                              \
    }


void phods_motion_estimation(const int current[N][M], const int previous[N][M],
        int vectors_x[N/Bx][M/By],int vectors_y[N/Bx][M/By])
{
  int x, y, min1, min2, bestx, besty;
  int vx, vy;   /*copies of vectors_x[x][y], vectors_y[x][y]*/
  int bx, by;   /*top-left pixel of the current block (Bx*x, By*y)*/
  int cb[Bx][By];   /*copy of the current block in RAM*/

  /*variables of the inner loops kept in registers*/
  register int k, l, d, p1;
  register int distx, disty, distc;
  register const int *cr;   /*row of the current block*/
  register const int *pr;   /*row of the previous frame (X candidate / central candidate)*/
  register const int *qr;   /*row of the previous frame (Y candidate)*/

  /*padded previous frame living in RAM*/
  static int prev_pad[N+2*p][M+2*p];

  /*Copy the previous frame*/
  for(k=N-1; k>=0; k--)
  {
    for(l=M-1; l>=0; l--)
    {
      prev_pad[k+p][l+p] = previous[k][l];
    }
  }

  /*For all blocks in the current frame*/
  for(x=((N/Bx)-1); x>=0; x--)
  {
    for(y=((M/By)-1); y>=0; y--)
    {
      bx = Bx*x;
      by = By*y;

      vx = 0;
      vy = 0;

      /*Copy the current block*/
      for(k=0; k<Bx; k++)
      {
        for(l=0; l<By; l++)
        {
          cb[k][l] = current[bx+k][by+l];
        }
      }

      s_loop(4);
      s_loop(2);
      s_loop(1);

      vectors_x[x][y] = vx;
      vectors_y[x][y] = vy;
    }
  }

}

/*******************************************************************************************************************//**
 * @brief  Blinky example application
 *
 * Blinks all leds at a rate of 1 second using the software delay function provided by the BSP.
 * Only references two other modules including the BSP, IOPORT.
 *
 **********************************************************************************************************************/
void hal_entry(void) {


	// Code to initialize the DWT->CYCCNT register
    CoreDebug->DEMCR |= 0x01000000;
       ITM->LAR = 0xC5ACCE55;
       DWT->CYCCNT = 0;
       DWT->CTRL |= 1;
	   /* Initialize your variables here */
     uint32_t cycles_i, cycles_in, cycles_out, min_cycles, max_cycles;
     uint32_t cycles[10];
     uint64_t total_cycles = 0;
     double min_time, max_time, avg_time;
     min_cycles = 0xFFFFFFFF;
     max_cycles = 0;


       int motion_vectors_x[N/Bx][M/By],
           motion_vectors_y[N/Bx][M/By], i, j;

      for(int idx=10-1; idx>=0; idx--){
		 /* Add timer code here */

         cycles_in = DWT->CYCCNT;
         phods_motion_estimation(current,previous,motion_vectors_x,motion_vectors_y);
         cycles_out = DWT->CYCCNT;
         cycles_i = cycles_out - cycles_in;
         cycles[idx] = cycles_i;
         if (cycles_i < min_cycles) min_cycles = cycles_i;
         if (cycles_i > max_cycles) max_cycles = cycles_i;
         total_cycles += cycles_i;

      }

      max_time = ((double)max_cycles / SystemCoreClock) * 1000;
      min_time = ((double)min_cycles / SystemCoreClock) * 1000;
      avg_time = (((double)total_cycles / 10) / SystemCoreClock) * 1000;

      while(1){

      }



}
