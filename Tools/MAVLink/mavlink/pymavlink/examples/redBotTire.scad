rimD = 52;
sideWall = 6;
width = 26;

difference() {
    cylinder(r=rimD/2 + sideWall, h=width);
    cylinder(r=rimD/2 + sideWall, h=width);
}
