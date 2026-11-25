# Reporting to the dashboard

Each subsystem reports to the Mission Control dashboard (`dashboard/`) with one function call. The Wi-Fi side is already wired: start-up, MQTT publishing, the `robot/command` subscription and the heartbeat. All you add is the call at the right moment in your own code.

```c
#include "../../subsystem1/wifi/dashboard_link.h"
```

## Rules

1. **Report events, not loop samples.** Call once when something happens (a barcode is decoded, a hump ends, an obstacle is profiled), not every 10 ms. The queue holds 8 messages.
2. **Never wait on the result.** Calls do not block. `E_TMOUT` (queue full) and `E_OBJ` (MQTT not built in) are safe to ignore, because the car must keep driving when telemetry is lost.
3. **Use millimetres.** Every distance argument is `uint16_t` mm. The dashboard converts to cm for display.

## What to call

| Owner | When | Call | Dashboard shows |
|---|---|---|---|
| Buddy 3 | Bars first seen under the barcode sensor | `dashboard_report_barcode_detected()` | Event log |
| Buddy 3 | Character decoded | `dashboard_report_barcode('B')` | Amber letter, next junction, recent barcodes |
| Buddy 2 | Junction turn finished | `dashboard_report_action_done(turn)` | Barcode marked EXECUTED |
| Buddy 2 | Car lifted and motors stopped | `dashboard_report_pickup()` | Red banner, run timer freezes |
| Buddy 2 | Vehicle state changes | `dashboard_set_vehicle_state(WIFI_TELEMETRY_VEHICLE_RUNNING)` | Heartbeat (not queued) |
| Buddy 4 | While on a hump, at most 5 Hz | `dashboard_report_hump_height(height_mm)` | Current height |
| Buddy 4 | Once, when leaving a hump | `dashboard_report_hump_peak(peak_mm)` | Peak height |
| Buddy 5 | Obstacle first detected | `dashboard_report_obstacle(distance_mm)` | Distance, phase SCANNING |
| Buddy 5 | Fine scan finished | `dashboard_report_obstacle_profile(width_mm, left_mm, right_mm, DASHBOARD_SIDE_LEFT)` | Width, clearances, bypass side |
| Buddy 5 | Avoidance phase changes | `dashboard_report_avoid_phase(DASHBOARD_AVOID_SEARCH)` | Obstacle card chip |
| Buddy 5 | Back on the line | `dashboard_report_line_rejoined()` | Phase REJOINED |
| Buddy 2 | Straddle steering pivots at a bend, a search ends, or the line is given up (the controller sends these) | `dashboard_report_line_event(DASHBOARD_LINE_PIVOT)` (`_SEARCH_FOUND`, `_SEARCH_CENTRED`, `_LOST`) | Line card: bend pivots, searches; timeline (centred searches are counted, not logged) |

Line state in telemetry (straddle sensors, the tape runs between them): while following it is FOUND, including pivots and searches, and LOST only once the car has given up; when not following, both sensors white is UNKNOWN (shown as "Between sensors?"), since the tape may be between them or absent; lifted is UNKNOWN.

## Junction turn (from the dashboard)

The dashboard maps each decoded letter to a turn and sends that turn back to the car. Read it at the junction:

```c
dashboard_turn_t turn = dashboard_next_junction();	/* LEFT until a barcode arrives */

switch (turn) {
case DASHBOARD_TURN_LEFT:     /* turn left 90°  */ break;
case DASHBOARD_TURN_RIGHT:    /* turn right 90° */ break;
case DASHBOARD_TURN_UTURN:    /* turn 180°      */ break;
case DASHBOARD_TURN_STRAIGHT: /* keep going     */ break;
}
dashboard_report_action_done(turn);
/* Optional: make each barcode apply to one junction only. */
dashboard_junction_reset();
```

- The default is **Turn left**. The turn returns to the default when the dashboard's **Start run** button is pressed or `dashboard_junction_reset()` is called.
- If the operator remaps the last decoded letter, the new turn is resent straight away, so always read the turn at the junction, not when the barcode is decoded.
- The car does not wait for the dashboard. If Wi-Fi is down, it uses whatever turn it last received, which is Turn left if it never received one.

The Start and Stop buttons can be read by the controller (single consumer):

```c
dashboard_run_command_t run;
if (dashboard_take_run_command(&run)) {
	/* run == DASHBOARD_RUN_START or DASHBOARD_RUN_STOP */
}
```

## Suggested places

- **Buddy 3:** `subsystem3/line_barcode/line_barcode_task.c`, right after the barcode message is sent with `tk_snd_mbf(g_mbf_barcode_events, ...)` succeeds: `(void)dashboard_report_barcode(barcode.payload[0]);`
- **Buddy 4:** where `hump_peak_m` is final, i.e. on the HUMP→FLAT transition: `(void)dashboard_report_hump_peak((uint16_t)(state->hump_peak_m * 1000.0f));`
- **Buddy 5:** when the `obstacle_event_message_t` with `VEHICLE_OBSTACLE_SCAN_FOUND` is built, use its `width_mm`, `left_mm` and `right_mm` fields.

## Testing without the dashboard

```
mosquitto_sub -h <broker> -t 'robot/#' -v
mosquitto_pub -h <broker> -t robot/command -m "$(printf '\xc3\x01')"   # junction: right
```

Without the car, `npm run preview` in `dashboard/` replays a simulated run. The firmware's message formats are checked on the car by `WIFI-U08` to `WIFI-U10` (`dashboard_link_test()`).

| File | Purpose |
|---|---|
| `dashboard_link.h` | The API above |
| `dashboard_msg.c/.h` | Exact wire formats, with no kernel calls |
| `dashboard_link.c` | Queue to the network task, junction and run-command state |
| `dashboard_link_owner.h` | Network task side, used only by `lib/liblwip/.../mqtt_utk.c` |
