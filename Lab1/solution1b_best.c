/* hal_entry.c - PHODS motion estimation, optimized (intended for -O0) */

#include "hal_data.h"
#include "stdio.h"
#include "string.h"
#include "images.h"

#define N 10    /* Frame dimension */
#define M 10    /* Frame dimension */
#define B 5     /* Block size */

/* #define p 7  -- removed: it replaces every identifier named "p" */

/* Larger than any possible SAD (max = 255*B*B), so the first candidate is always selected */
#define LIMIT (255*B*B + 1)

#define CLAMP(v, lo, hi) (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))

/*
 * SAD of the current block against the previous frame at offset (dx,dy),
 * pixels outside the frame count as 0.
 *
 * - Valid rows are [k0,k1), valid columns are [l0,l1): computed once per candidate,
 *   so the inner loop has NO boundary checks.
 * - Out-of-frame pixels contribute |cur - 0| = cur, so
 *       SAD = (sum of whole block) - (sum of cur over valid rectangle)
 *             + sum |cur - prev| over valid rectangle
 *   The rectangle sums come from the prefix-sum table P (computed once per block).
 */
#define SAD(dx, dy, res)                                                    \
    do {                                                                    \
        cx = bx + (dx);                                                     \
        cy = by + (dy);                                                     \
        k0 = CLAMP(-cx, 0, B);                                              \
        k1 = CLAMP(N - cx, k0, B);                                          \
        l0 = CLAMP(-cy, 0, B);                                              \
        l1 = CLAMP(M - cy, l0, B);                                          \
        acc = P[B][B] - (P[k1][l1] - P[k0][l1] - P[k1][l0] + P[k0][l0]);    \
        for (k = k0; k < k1; k++)                                           \
        {                                                                   \
            cbr = cb[k];                                                    \
            pp  = previous[cx + k];                                         \
            for (l = l0; l < l1; l++)                                       \
            {                                                               \
                d = cbr[l] - pp[cy + l];                                    \
                acc += (d < 0) ? -d : d;                                    \
            }                                                               \
        }                                                                   \
        (res) = acc;                                                        \
    } while (0)

/*
 * One PHODS step with search distance S (S is a literal: 4, 2, 1).
 * - SAD(0,0) is shared by the X and Y searches (computed once, or reused
 *   from the previous step when the new centre was already a candidate).
 * - Candidate order -S, 0, +S with strict '<' keeps the same tie-breaking as the original.
 */
#define STEP(S)                                                             \
    do {                                                                    \
        if (cen_ok) { c = cen; } else { SAD(ax, ay, c); }                   \
        SAD(ax - (S), ay, sxm);                                             \
        SAD(ax + (S), ay, sxp);                                             \
        SAD(ax, ay - (S), sym);                                             \
        SAD(ax, ay + (S), syp);                                             \
                                                                            \
        min1 = LIMIT; bestx = 0;                                            \
        if (sxm < min1) { min1 = sxm; bestx = -(S); }                       \
        if (c   < min1) { min1 = c;   bestx = 0;    }                       \
        if (sxp < min1) { min1 = sxp; bestx = (S);  }                       \
                                                                            \
        min2 = LIMIT; besty = 0;                                            \
        if (sym < min2) { min2 = sym; besty = -(S); }                       \
        if (c   < min2) { min2 = c;   besty = 0;    }                       \
        if (syp < min2) { min2 = syp; besty = (S);  }                       \
                                                                            \
        ax += bestx;                                                        \
        ay += besty;                                                        \
                                                                            \
        /* SAD of the new centre, if it was already evaluated */            \
        if (besty == 0)      { cen = min1; cen_ok = 1; }                    \
        else if (bestx == 0) { cen = min2; cen_ok = 1; }                    \
        else                 { cen_ok = 0; }                                \
    } while (0)


void phods_motion_estimation(const int current[N][M], const int previous[N][M],
        int vectors_x[N/B][M/B], int vectors_y[N/B][M/B])
{
    int x, y, bx, by, ax, ay;
    int cx, cy, k0, k1, l0, l1;
    int c, cen, cen_ok, sxm, sxp, sym, syp;
    int min1, min2, bestx, besty, rowsum, v;
    int cb[B][B];          /* local copy of the current block (data reuse) */
    int P[B+1][B+1];       /* 2-D prefix sums of the current block          */
    const int *cr;

    register int k, l, d, acc;
    register const int *pp;
    register int *cbr;

    /* For all blocks in the current frame */
    for (x = 0; x < N/B; x++)
    {
        for (y = 0; y < M/B; y++)
        {
            bx = B * x;
            by = B * y;
            ax = 0;            /* scalar replacement of vectors_x[x][y] */
            ay = 0;            /* scalar replacement of vectors_y[x][y] */
            cen_ok = 0;
            cen = 0;

            /* Load the block once and build the prefix sums */
            for (l = 0; l <= B; l++) { P[0][l] = 0; }
            for (k = 0; k < B; k++)
            {
                cr = current[bx + k] + by;
                rowsum = 0;
                P[k+1][0] = 0;
                for (l = 0; l < B; l++)
                {
                    v = cr[l];
                    cb[k][l] = v;
                    rowsum += v;
                    P[k+1][l+1] = P[k][l+1] + rowsum;
                }
            }

            STEP(4);
            STEP(2);
            STEP(1);

            vectors_x[x][y] = ax;
            vectors_y[x][y] = ay;
        }
    }
}

void hal_entry(void) {

    // Code to initialize the DWT->CYCCNT register
    CoreDebug->DEMCR |= 0x01000000;
    ITM->LAR = 0xC5ACCE55;
    DWT->CYCCNT = 0;
    DWT->CTRL |= 1;

    uint32_t cycles_i, cycles_in, cycles_out, min_cycles, max_cycles;
    uint32_t cycles[10];
    uint64_t total_cycles = 0;
    double min_time, max_time, avg_time;
    min_cycles = 0xFFFFFFFF;
    max_cycles = 0;

    int motion_vectors_x[N/B][M/B],
        motion_vectors_y[N/B][M/B];

    for (int idx = 0; idx < 10; idx++) {
        cycles_in = DWT->CYCCNT;
        phods_motion_estimation(current, previous, motion_vectors_x, motion_vectors_y);
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

    while (1) {
    }
}