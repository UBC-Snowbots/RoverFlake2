# Snack Run — 6408-M Vending Machine

Source: Vending Machine Technician Manual rev 1.1 (2026-07-19).

## Panel layout
- **Top-left:** CreditCube tap + Credits display below it
- **Left, below tap:** selection buttons `A B C D 1 2 3 4` + `Clear` + `Enter`, Selection display above
- **Middle:** 8 item compartments, selection-code display under each (doubles as error display)
- **Below compartments:** dispensing slot; **right of slot:** service panel (switches)
- **Lower-left:** auxiliary power core slot · **Right:** telemetry/licensing antenna

## Operation
- **Credits:** only ONE cube face works (different color or recessed cube symbol). Hold on tap pad; 1 credit per **5 s**. Remove cube to stop.
- **Selection:** Clear display → press code under compartment → Enter. Buttons register on **press**, not release. Full display ignores input until Clear/Enter. Enter is ignored with no credits or invalid selection.

## Known display quirks
- Pre-**M4a**: display cycles through multiple selection codes — enter one of the shown codes while it's displayed.
- Pre-**M3c**: display shows **raw binary ASCII** instead of characters → use the HMI converter.

## Error codes (`ErrX` on selection displays)
| Code | Fault | Fix |
|---|---|---|
| Err0 | Aux power failure | Insert power core, lower-left slot. **Contact end (small colored cap) goes in first.** |
| Err1 | Locked item | Not a fault. Manufacturer bypass code: **4321** |
| Err2 | Hardware fault | See decode procedure below |
| Err3 | Bad telemetry | Clean antenna with the loop (stored at antenna base; hangs on hook at top when done). **Minimize loop↔antenna contact** — excessive contact destroys the unit. |
| Err4 | Jammed item | Percussive maintenance: 2 light taps on top of chassis, **≤1 s apart**. May need 2–5 double-taps. No shaking/slamming. |

## Err2 decode procedure
1. Open service panel: rotate handle 90° CW, pull cover.
2. After `Err2`, the display cycles 7-seg patterns encoding bytes as `0b0gfedcba` (bit per segment). Each byte is an ASCII char.
3. Rows of 4 bytes; **XOR each column** down the rows → 4 bytes → ASCII, e.g. `UddU` = switches up, down, down, up.
4. Set the service panel switches to that sequence; verify error clears.
- Note: Err2 can trip spuriously from dust in the smoke detector — if nothing is smoking, skip board replacement.
