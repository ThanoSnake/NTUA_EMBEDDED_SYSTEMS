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

/*SIMD SAD WITH THE CORTEX-M4 DSP INSTRUCTION USADA8*/
/*USADA8(a, b, acc) = acc + |a.b0-b.b0| + |a.b1-b.b1| + |a.b2-b.b2| + |a.b3-b.b3| (4 unsigned bytes, 1 cycle).
  - The copies cb and prev_pad are now uint8_t, so 4 neighbouring pixels are loaded with one 32-bit load
    (unaligned loads are supported by the Cortex-M4).
  - USADA8 works on unsigned bytes. The pixels of images.h are signed (-128..124), so BIAS=128 is added during
    the copy: |(a+128)-(b+128)| = |a-b|, the SAD (and the motion vectors) are exactly the same.
    For normal 0..255 images set BIAS to 0.
  - The padding represents pixels with value 0, so it is filled with 0+BIAS.
  - The columns of a row are processed 4 at a time, the remaining By%4 columns with the scalar code,
    so the code works for any Bx, By.
  - Memory: prev_pad and cb are 4 times smaller than with int*/

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

#define BIAS 128          /*signed pixels -> unsigned bytes (0 for 0..255 images)*/
#define PAD  (0 + BIAS)   /*value of a pixel outside the frame*/

/*Unaligned 32-bit load (4 pixels). The packed struct makes GCC emit a single LDR that is allowed
  to be unaligned (same technique as CMSIS __UNALIGNED_UINT32_READ)*/
typedef struct __attribute__((packed)) { uint32_t v; } ua_u32_t;
#define LOAD4(ptr) (((const ua_u32_t *)(ptr))->v)


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
    for(k=0; k<Bx; k++)                                                         \
    {                                                                           \
        cr = cb[k];                                                             \
        pr = &prev_pad[bx+vx+k+p][by+vy+p];                                     \
        for(l=0; l+4<=By; l+=4)                                                 \
        {                                                                       \
            distc = __USADA8(LOAD4(cr+l), LOAD4(pr+l), distc);                  \
        }                                                                       \
        for(; l<By; l++)                                                        \
        {                                                                       \
            d = cr[l]-pr[l];                                                    \
            distc += (d<0) ? -d : d;                                            \
        }                                                                       \
    }                                                                           \
    if((int)distc < min1)                                                       \
    {                                                                           \
        min1 = distc;                                                           \
        bestx = 0;                                                              \
    }                                                                           \
    if((int)distc < min2)                                                       \
    {                                                                           \
        min2 = distc;                                                           \
        besty = 0;                                                              \
    }


/*Side candidates (i=-s, i=+s). X and Y search merged in one loop*/
#define i_loop(i)                                                               \
    distx = 0;                                                                  \
    disty = 0;                                                                  \
    for(k=0; k<Bx; k++)                                                         \
    {                                                                           \
        cr = cb[k];                                                             \
        pr = &prev_pad[bx+vx+i+k+p][by+vy+p];   /*row of the X candidate*/      \
        qr = &prev_pad[bx+vx+k+p][by+vy+i+p];   /*row of the Y candidate*/      \
        for(l=0; l+4<=By; l+=4)                                                 \
        {                                                                       \
            c4 = LOAD4(cr+l);                                                   \
            distx = __USADA8(c4, LOAD4(pr+l), distx);                           \
            disty = __USADA8(c4, LOAD4(qr+l), disty);                           \
        }                                                                       \
        for(; l<By; l++)                                                        \
        {                                                                       \
            p1 = cr[l];                                                         \
            d = p1-pr[l];                                                       \
            distx += (d<0) ? -d : d;                                            \
            d = p1-qr[l];                                                       \
            disty += (d<0) ? -d : d;                                            \
        }                                                                       \
    }                                                                           \
    if((int)distx < min1)                                                       \
    {                                                                           \
        min1 = distx;                                                           \
        bestx = i;                                                              \
    }                                                                           \
    if((int)disty < min2)                                                       \
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
  uint8_t cb[Bx][By];   /*copy of the current block in RAM (biased bytes)*/

  /*variables of the inner loops kept in registers*/
  register int k, l, d, p1;
  register uint32_t distx, disty, distc;   /*USADA8 accumulators*/
  register uint32_t c4;                    /*4 pixels of the current block*/
  register const uint8_t *cr;   /*row of the current block*/
  register const uint8_t *pr;   /*row of the previous frame (X candidate / central candidate)*/
  register const uint8_t *qr;   /*row of the previous frame (Y candidate)*/

  /*padded previous frame living in RAM. The border is filled with PAD once at startup
    (GCC range initializer, the array goes to .data) and is never written again*/
  static uint8_t prev_pad[N+2*p][M+2*p] =
      { [0 ... N+2*p-1] = { [0 ... M+2*p-1] = PAD } };

  /*Copy the previous frame (biased)*/
  for(k=0; k<N; k++)
  {
    for(l=0; l<M; l++)
    {
      prev_pad[k+p][l+p] = (uint8_t)(previous[k][l] + BIAS);
    }
  }

  /*For all blocks in the current frame*/
  for(x=0; x<N/Bx; x++)
  {
    for(y=0; y<M/By; y++)
    {
      bx = Bx*x;
      by = By*y;

      vx = 0;
      vy = 0;

      /*Copy the current block (biased)*/
      for(k=0; k<Bx; k++)
      {
        for(l=0; l<By; l++)
        {
          cb[k][l] = (uint8_t)(current[bx+k][by+l] + BIAS);
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

      for(int idx=0; idx<10; idx++){
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
