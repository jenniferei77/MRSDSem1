/*
 * Simple test case
 *
 * 15-122 Principles of Imperative Computation */


#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <assert.h>
#include "lib/xalloc.h"
#include "simple.h"

int main() {
  struct point2d* P = xmalloc(sizeof(struct point2d));
  P->x = -15;
  P->y = 0; // uninitialized without: P->y could contain an arbitrary value
  P->y = P->y + absval(P->x * 2);
  assert(P->y > P->x && true);   // if P->y uninitialized, may succeeed or fail

  // free(P);  // wrong!
  printf("x coord: %d\n", P->x);
  printf("y coord: %d\n", P->y);  // if P->y uninitialized, may print anything

  free(P);
  // free(P);  // wrong!

  return 0;
}
