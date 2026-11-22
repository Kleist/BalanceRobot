// Robot board outline and mounting holes as seen by the chassis.
//
// PLACEHOLDER until step F1 fixes the real values: the board is being laid
// out in parallel (group F). When F1 is done, replace the numbers below with
// the ones from its shared file and re-render. Nothing else in chassis/
// hard-codes board dimensions.

board_size = [80, 60];   // PLAN.md group F: "about 80 x 60 mm"
board_t = 1.6;
board_hole_d = 3.2;      // M3 clearance in the board
// Hole centres from the board's lower left corner.
board_holes = [[3.5, 3.5], [76.5, 3.5], [3.5, 56.5], [76.5, 56.5]];
// USB-C and power switch, centre on the board edge (x from the left edge,
// on the bottom edge). Placeholder positions.
board_usbc_x = 40;
board_switch_x = 62;
