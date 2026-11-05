/** @file s5_edge.h @brief Finds where an obstacle ENDS while the car drives along it. */
#ifndef S5_EDGE_H
#define S5_EDGE_H

#include <stdbool.h>
#include <stdint.h>

/* Start looking for the end of the obstacle (call as the move starts).
 * b_wait_for_obstacle = true when the obstacle is still AHEAD of the car and
 * will only come into view beside it as the car drives (passing it): nothing
 * seen at first then means "not there yet", not "already clear", and the wait
 * is given up after wait_limit_ticks.  false when the obstacle is already
 * beside the car at the start. */
void s5_edge_start (const bool b_wait_for_obstacle,
                    const uint32_t wait_limit_ticks);

/*
 * Feed one reading taken with the sensor looking straight at the obstacle.
 * distance_cm = S5_OBSTACLE_NONE_CM when nothing echoed; b_good = false when
 * the reading failed (it says nothing and is ignored); moved_ticks is how far
 * the move has gone.  The first reading fixes how far away the obstacle's
 * face is; the obstacle has ENDED once S5_EDGE_CONFIRM readings in a row no
 * longer show it.  Returns true once it has ended.
 */
bool s5_edge_feed (const bool b_good, const uint16_t distance_cm,
                   const uint32_t moved_ticks);

/* How far the move had gone when the obstacle was last seen (ticks). */
uint32_t s5_edge_last_seen_ticks (void);

#endif
