// Workpiece with exactly two diagonal blind holes
// All dimensions are in millimetres

difference() {
    // Solid workpiece: 300 x 300 x 300 mm
    cube([300, 300, 300], center=true);

    // Hole 1: diagonal position
    translate([-80, 80, -1])
        cylinder(h=152, d=25, $fn=96);

    // Hole 2: opposite diagonal position
    translate([80, -80, -1])
        cylinder(h=152, d=25, $fn=96);
}
