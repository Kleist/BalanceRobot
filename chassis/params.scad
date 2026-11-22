// Shared parameters for the chassis (group G). All lengths in mm.
//
// Every dimension marked "MEASURE" is a best guess for the common yellow
// TT gear motor and must be checked with calipers on the real parts before
// the print that depends on it. See README.md for the list.

// ---------------------------------------------------------------- printer
// Prusa MK3, 0.4 mm nozzle, 0.2 mm layers, PLA (first prints), PETG later.
nozzle = 0.4;
layer = 0.2;
// Clearance between two printed parts that should slide together.
// The tolerance comb on the fit-test plate (G1) measures the real value.
fit_slide = 0.3;
// Clearance between a printed hole and a bought part (motor body, cell).
fit_part = 0.4;
// Horizontal holes print slightly small and oval: add this to their diameter.
hole_extra = 0.2;

// ---------------------------------------------------------------- fasteners
m3_clear = 3.4;         // M3 screw passes freely
m3_nut_af = 5.5;        // M3 hex nut across flats
m3_nut_t = 2.4;         // M3 hex nut thickness
m3_head_d = 5.5;        // M3 socket head diameter
// M3 heat-set inserts: the hole diameter is what G1 decides.
// Defaults suit ruthex-style M3 x 5.7 inserts (OD ~4.6, hole 4.0).
insert_hole_d = 4.0;
insert_depth = 6.0;     // insert length + 0.3 mm so it doesn't bottom out
insert_wall = 1.6;      // plastic around the insert (4 perimeters)
insert_test_d = [3.8, 4.0, 4.2];  // the three sizes on the fit-test plate

// ---------------------------------------------------------------- TT motor
// Axes of the motor model: X along the motor (0 = gearbox end away from the
// can, +X towards the can), Y along the output shaft (0 = one flat face of
// the gearbox), Z up (0 = gearbox bottom edge).
tt_gb_len = 37.0;       // MEASURE gearbox length, end to where the can starts
tt_gb_h = 22.4;         // MEASURE gearbox height (perpendicular to shaft and length)
tt_body_t = 18.8;       // MEASURE gearbox thickness along the shaft, face to face
tt_shaft_x = 11.2;      // MEASURE shaft centre from the gearbox end
tt_shaft_z = tt_gb_h / 2;  // MEASURE shaft centre from the bottom edge
tt_can_d = 20.0;        // MEASURE motor can diameter (round part)
tt_can_len = 26.0;      // MEASURE can length incl. the rear cap with the tabs
tt_shaft_d = 5.4;       // MEASURE shaft diameter over the round part
tt_shaft_flats = 3.7;   // MEASURE shaft across the two flats
tt_shaft_len = 9.0;     // MEASURE shaft length out of one gearbox face
tt_shaft_hole_d = 2.0;  // MEASURE hole in the shaft end (for the wheel screw)
tt_boss_d = 7.0;        // MEASURE round boss around the shaft on the gearbox face
tt_boss_h = 1.0;        // MEASURE how far that boss stands out
// The two mounting holes run through the gearbox parallel to the shaft.
tt_holes = [[31.5, 2.8], [31.5, 19.6]];  // MEASURE [x, z] of each hole centre
tt_hole_d = 3.0;        // MEASURE mounting hole diameter

// ---------------------------------------------------------------- 18650 cell
cell_d = 18.6;          // MEASURE your cells (protected cells can be ~18.9)
cell_len = 65.2;        // MEASURE (protected cells can be ~69)

// ---------------------------------------------------------------- USB-C
// The plug overmold of the cable is what has to reach the board's receptacle.
usbc_plug = [12.5, 6.5];          // typical overmold, width x height
usbc_test = [[12.5, 6.5], [13.2, 7.0], [14.0, 7.6]];
// Power slide switch actuator slot: depends on the switch F2 picks.
switch_test = [[7.0, 3.0], [8.0, 3.5], [9.0, 4.0]];

// ---------------------------------------------------------------- labels
label_font = "DejaVu Sans:style=Bold";
label_size = 4;
label_depth = 0.6;      // 3 layers, debossed into a top face
