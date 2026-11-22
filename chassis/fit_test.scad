// G1: fit-test plate. One PLA print that answers the fit questions before
// the first body is printed. See README.md for what each piece tests and
// what to write down.
//
// Render everything:     openscad -o fit_test.stl fit_test.scad
// Render one piece:      openscad -D 'part="cradle"' -o cradle.stl fit_test.scad
// Show the motor/board ghosts in the preview (F5 in the GUI): ghosts = true.

include <params.scad>
include <board.scad>

part = "plate";   // plate | cradle | hubs | inserts | board | ports | cell | comb
ghosts = false;   // draw the bought parts (not printed) in the preview

$fn = 64;
eps = 0.01;

// ------------------------------------------------------------------ helpers

module label(txt, size = label_size) {
    // Debossed text, placed with its baseline centre at the origin, on a top
    // face at z = 0 (cuts down by label_depth).
    translate([0, 0, -label_depth])
        linear_extrude(label_depth + eps)
            text(txt, size = size, font = label_font, halign = "center", valign = "center");
}

module rounded_rect(size, r) {
    // 2D rectangle centred on the origin with rounded corners.
    r2 = min(r, size[0] / 2 - eps, size[1] / 2 - eps);
    offset(r = r2) square([size[0] - 2 * r2, size[1] - 2 * r2], center = true);
}

module double_d(d, flats) {
    // 2D cross-section of the TT shaft: a circle with two parallel flats.
    intersection() {
        circle(d = d);
        square([flats, d + 1], center = true);
    }
}

module hex_pocket(af, depth) {
    // Hex nut pocket, across flats af, along +Z.
    cylinder(d = af / cos(30), h = depth, $fn = 6);
}

// ------------------------------------------------------------------ ghosts

module tt_motor() {
    // Rough model of the TT gear motor in its own axes (see params.scad).
    color("gold", 0.6) difference() {
        cube([tt_gb_len, tt_body_t, tt_gb_h]);
        for (h = tt_holes) translate([h[0], -1, h[1]]) rotate([-90, 0, 0])
            cylinder(d = tt_hole_d, h = tt_body_t + 2);
    }
    color("silver", 0.6) translate([tt_gb_len, tt_body_t / 2, tt_shaft_z]) rotate([0, 90, 0])
        cylinder(d = tt_can_d, h = tt_can_len);
    color("white", 0.8) translate([tt_shaft_x, -tt_shaft_len, tt_shaft_z]) rotate([-90, 0, 0])
        linear_extrude(tt_body_t + 2 * tt_shaft_len) double_d(tt_shaft_d, tt_shaft_flats);
}

// ------------------------------------------------------------------ 1. motor cradle

cradle_floor = 3;
cradle_wall = 3;
cradle_inner = tt_body_t + fit_part;       // gap between the walls
cradle_w = cradle_inner + 2 * cradle_wall;
cradle_len = tt_gb_len + tt_can_len;
cradle_wall_h = tt_gb_h;                    // walls as high as the gearbox
shaft_clear_d = max(tt_boss_d, tt_shaft_d) + 2;

module cradle() {
    // Holds the whole motor: two walls along the gearbox with the two M3
    // through-bolts (M3 x 30 + nut), a saddle under the can, and a zip-tie
    // slot for the motor leads behind the can.
    // Motor sits at x = 0..cradle_len, y = cradle_wall.., z = cradle_floor..
    difference() {
        union() {
            // floor
            cube([cradle_len, cradle_w, cradle_floor]);
            // gearbox walls
            for (y = [0, cradle_w - cradle_wall])
                translate([0, y, 0]) cube([tt_gb_len, cradle_wall, cradle_floor + cradle_wall_h]);
            // can saddle, up to the can's centre line
            translate([tt_gb_len, 0, 0])
                cube([tt_can_len - 4, cradle_w, cradle_floor + tt_shaft_z]);
        }
        // can seat
        translate([tt_gb_len - eps, cradle_w / 2, cradle_floor + tt_shaft_z]) rotate([0, 90, 0])
            cylinder(d = tt_can_d + fit_part, h = tt_can_len);
        // gearbox pocket in the saddle's end, in case the gearbox is longer
        // than measured: the motor still slides in
        translate([tt_gb_len - eps, cradle_wall, cradle_floor])
            cube([1, cradle_inner, tt_gb_h]);
        // shaft + boss clearance through both walls
        translate([tt_shaft_x, -1, cradle_floor + tt_shaft_z]) rotate([-90, 0, 0])
            cylinder(d = shaft_clear_d, h = cradle_w + 2);
        // M3 through-bolts
        for (h = tt_holes) translate([h[0], -1, cradle_floor + h[1]]) rotate([-90, 0, 0])
            cylinder(d = m3_clear + hole_extra, h = cradle_w + 2);
        // nut pockets on the outside of the far wall (hex, flat on top so it bridges)
        for (h = tt_holes) translate([h[0], cradle_w - 2.4 + eps, cradle_floor + h[1]])
            rotate([-90, 0, 0]) rotate([0, 0, 30]) hex_pocket(m3_nut_af + 0.3, 2.4);
        // zip-tie slots for the motor leads, through the floor behind the saddle
        for (y = [cradle_w / 2 - 6, cradle_w / 2 + 6 - 2.2])
            translate([cradle_len - 3.5, y, -1]) cube([2.2, 2.2, cradle_floor + 2]);
        // label on the outside of the near wall's top
        translate([tt_gb_len / 2, cradle_wall / 2, cradle_floor + cradle_wall_h])
            label("TT", size = 2.8);
    }
    if (ghosts) translate([0, cradle_wall + fit_part / 2, cradle_floor]) tt_motor();
}

// ------------------------------------------------------------------ 2. wheel hubs on the shaft

hub_clearances = [0.0, 0.1, 0.2, 0.3];
hub_od = 13;
hub_h = 9;
hub_cap = 1.6;      // closed end with the hole for the screw into the shaft
hub_pitch = 18;

module hub(c) {
    // A hub that slides onto the D-shaft; the screw goes through the cap into
    // the shaft's centre hole. Printed cap down.
    difference() {
        cylinder(d = hub_od, h = hub_h);
        translate([0, 0, hub_cap]) linear_extrude(hub_h)
            double_d(tt_shaft_d + c, tt_shaft_flats + c);
        translate([0, 0, -1]) cylinder(d = tt_shaft_hole_d + 0.6, h = hub_cap + 2);
        // 0.6 mm chamfer around the D at the open end so the shaft finds its way
        hull() {
            translate([0, 0, hub_h - 0.6]) linear_extrude(eps) double_d(tt_shaft_d + c, tt_shaft_flats + c);
            translate([0, 0, hub_h]) linear_extrude(eps) offset(delta = 0.6) double_d(tt_shaft_d + c, tt_shaft_flats + c);
        }
    }
}

module hubs() {
    // Four hubs on a tab that names their clearance. Snap them off with pliers.
    tab = [hub_pitch * len(hub_clearances), 9, 1.2];
    difference() {
        translate([-hub_pitch / 2, -hub_od / 2 - 1.5 - tab[1], 0]) cube(tab);
        for (i = [0 : len(hub_clearances) - 1])
            translate([i * hub_pitch, -hub_od / 2 - 1.5 - tab[1] / 2, tab[2]])
                label(str(hub_clearances[i] * 10), size = 4);
    }
    for (i = [0 : len(hub_clearances) - 1]) translate([i * hub_pitch, 0, 0]) {
        hub(hub_clearances[i]);
        // thin bridge to the tab
        translate([-1.5, -hub_od / 2 - 2, 0]) cube([3, 2.6, 0.6]);
    }
}

// ------------------------------------------------------------------ 3. heat-set inserts

insert_pitch = 11;

module inserts() {
    // Three insert holes; press in one insert each with the soldering iron
    // (~220 C for PLA) and see which sits flush and holds.
    n = len(insert_test_d);
    size = [n * insert_pitch + 2, 18, insert_depth + 2];
    difference() {
        cube(size);
        for (i = [0 : n - 1]) {
            d = insert_test_d[i];
            translate([insert_pitch * (i + 0.5) + 1, 6.5, size[2] - insert_depth]) {
                cylinder(d = d, h = insert_depth + eps);
                // 0.5 mm lead-in chamfer
                translate([0, 0, insert_depth - 0.5]) cylinder(d1 = d, d2 = d + 1, h = 0.5 + 2 * eps);
            }
            translate([insert_pitch * (i + 0.5) + 1, 14, size[2]]) label(str(floor(d), ".", round(d * 10) % 10), size = 3.2);
        }
    }
}

// ------------------------------------------------------------------ 4. board hole pattern

standoff_h = 6;
standoff_od = insert_hole_d + 2 * insert_wall + 0.4;

module board_test() {
    // A thin frame the size of the board with a standoff and an insert at each
    // mounting hole. Lay the real (or a cardboard) board on it.
    frame_w = 5;
    difference() {
        linear_extrude(1.2) difference() {
            square(board_size);
            translate([frame_w, frame_w]) square(board_size - [2 * frame_w, 2 * frame_w]);
        }
        translate([board_size[0] / 2, 2.5, 1.2]) label("BOARD", size = 3);
    }
    for (h = board_holes) translate(h) difference() {
        cylinder(d = standoff_od, h = standoff_h);
        translate([0, 0, standoff_h - insert_depth]) cylinder(d = insert_hole_d, h = insert_depth + eps);
    }
    // diagonal ribs so the frame stays flat when it comes off the bed
    for (d = [[[0, 0], board_size], [[board_size[0], 0], [0, board_size[1]]]])
        linear_extrude(1.2) intersection() {
            square(board_size);
            hull() for (q = d) translate(q) circle(d = 5);
        }
    if (ghosts) color("green", 0.5) translate([0, 0, standoff_h]) difference() {
        cube([board_size[0], board_size[1], board_t]);
        for (h = board_holes) translate([h[0], h[1], -1]) cylinder(d = board_hole_d, h = board_t + 2);
    }
}

// ------------------------------------------------------------------ 5. USB-C and switch cut-outs

port_wall = 2;     // the body wall the plug passes through

module ports() {
    // A piece of body wall, printed flat: three USB-C plug openings and three
    // switch slots. Push your cable's plug and the switch lever through.
    w = 74;
    h = 32;
    difference() {
        cube([w, h, port_wall]);
        for (i = [0 : 2]) {
            o = usbc_test[i];
            translate([13 + i * 24, 22, -1]) linear_extrude(port_wall + 2) rounded_rect(o, o[1] / 2);
            translate([13 + i * 24, 13.5, port_wall]) label(str(i + 1), size = 3);
        }
        for (i = [0 : 2]) {
            s = switch_test[i];
            translate([13 + i * 24, 6, -1]) linear_extrude(port_wall + 2) rounded_rect(s, 0.5);
        }
    }
}

// ------------------------------------------------------------------ 6. cell seat

seat_len = 24;
seat_wall = 2.4;

module cell_seat() {
    // A short saddle for one 18650 cell with two insert holes for a strap or
    // lid. Tests the cell diameter fit and the insert holes in a thin rib.
    d = cell_d + fit_part;
    w = d + 2 * (seat_wall + standoff_od);
    hgt = d / 2 + 2;
    difference() {
        translate([-w / 2, 0, 0]) cube([w, seat_len, hgt]);
        translate([0, -1, d / 2 + 2]) rotate([-90, 0, 0]) cylinder(d = d, h = seat_len + 2);
        for (x = [-1, 1]) translate([x * (w / 2 - standoff_od / 2), seat_len / 2, hgt - insert_depth])
            cylinder(d = insert_hole_d, h = insert_depth + eps);
    }
    if (ghosts) color("cyan", 0.5) translate([0, 0, d / 2 + 2]) rotate([-90, 0, 0])
        cylinder(d = cell_d, h = seat_len);
}

// ------------------------------------------------------------------ 7. tolerance comb

comb_clearances = [0.1, 0.2, 0.3, 0.4, 0.5];
comb_pin = 6;
comb_pitch = 11;

module comb() {
    // Round and square holes 6 mm + 0.1..0.5 mm, plus a loose round and square
    // pin. The smallest hole each pin slides through by hand = fit_slide.
    n = len(comb_clearances);
    size = [n * comb_pitch + 2, 24, 5];
    difference() {
        cube(size);
        for (i = [0 : n - 1]) {
            c = comb_clearances[i];
            x = comb_pitch * (i + 0.5) + 1;
            translate([x, 17, -1]) cylinder(d = comb_pin + c, h = size[2] + 2);
            translate([x, 7, size[2] / 2]) cube([comb_pin + c, comb_pin + c, size[2] + 2], center = true);
            translate([x, 21.8, size[2]]) label(str(c), size = 2.8);
        }
    }
    // the pins, beside the comb
    translate([size[0] + 6, 17, 0]) cylinder(d = comb_pin, h = 12);
    translate([size[0] + 6 - comb_pin / 2, 7 - comb_pin / 2, 0]) cube([comb_pin, comb_pin, 12]);
}

// ------------------------------------------------------------------ plate

module plate() {
    // Laid out for the MK3 bed (250 x 210), everything flat on the bed.
    translate([5, 5, 0]) board_test();
    translate([95, 5, 0]) cradle();
    translate([95, 40, 0]) ports();
    translate([180, 5, 0]) inserts();
    translate([204, 40, 0]) cell_seat();
    translate([110 + hub_od / 2, 95, 0]) hubs();
    translate([5, 75, 0]) comb();
}

if (part == "plate") plate();
else if (part == "cradle") cradle();
else if (part == "hubs") hubs();
else if (part == "inserts") inserts();
else if (part == "board") board_test();
else if (part == "ports") ports();
else if (part == "cell") cell_seat();
else if (part == "comb") comb();
