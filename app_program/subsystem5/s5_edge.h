/**
 * @file    s5_edge.h
 * @brief   Finds where an obstacle ENDS while the car drives along it.
 */
#ifndef S5_EDGE_H
#define S5_EDGE_H

#include <stdbool.h>
#include <stdint.h>

/* Start looking for the end of the obstacle (call as the move starts).
 * b_wait_for_obstacle = true when passing it: while nothing (or only an echo
 * further than S5_EDGE_NEAR_CM) has been seen beside the car, it is taken
 * as a short obstacle that has already ended, until it does come into view
 * (then it is followed as usual).  false when the obstacle is already in
 * view at the start (driving out). */
void s5_edge_start (const bool b_wait_for_obstacle);

/*
 * Feed one reading taken with the sensor looking straight at the obstacle.
 * distance_cm = S5_OBSTACLE_NONE_CM when nothing echoed; b_good = false when
 * the reading failed (it says nothing and is ignored); moved_ticks is how far
 * the move has gone.  The first reading fixes how far away the obstacle's
 * face is; the obstacle has ENDED once S5_EDGE_CONFIRM readings in a row no
 * longer show it.  Each reading that still shows it becomes the new face
 * distance, so a slow drift (heading a little off) is followed.  Returns
 * true while it has ended.  It goes back to false
 * if the obstacle is seen beside the car again (within S5_EDGE_TOL_CM of the
 * face, or within S5_EDGE_NEAR_CM if no face was seen at first), so the
 * caller keeps driving instead of turning in.
 */
bool s5_edge_feed (const bool b_good, const uint16_t distance_cm,
                   const uint32_t moved_ticks);

/* How far the move had gone when the obstacle was last seen (ticks). */
uint32_t s5_edge_last_seen_ticks (void);


/* True once the obstacle has been seen beside the car at least once since
 * s5_edge_start().  False means the end was only assumed (nothing seen at
 * all, or the wait ran out), so the move went by distance alone. */
bool s5_edge_was_seen (void);

#endif
