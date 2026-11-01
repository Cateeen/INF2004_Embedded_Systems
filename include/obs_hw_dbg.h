/* obs_hw_dbg.h - debug helpers shared by obs_hw_main.c and obs_port_pico_bare.c (bench only) */
#ifndef OBS_HW_DBG_H
#define OBS_HW_DBG_H

void obs_hw_clear_table(void);   /* forget the per-angle distances (start of a new scan) */
void obs_hw_print_table(void);   /* angle vs distance table with bars                    */
void obs_hw_print_stats(void);   /* measurement counters, pulse min/max, rise delay      */
void obs_hw_help(void);          /* defined in obs_hw_main.c                             */

#endif