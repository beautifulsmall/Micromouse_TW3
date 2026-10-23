// The rolling search's path geometry (Explore.h): the in-cell turn (an arc
// with clothoid ramps between two straights), the straight-through cell,
// and the rolling extension, checked on SpeedPath alone -- the mission
// itself needs the mouse. What this pins down is the arithmetic the cell
// records are built from: where the next cell's turn point lands after
// every kind of extension, and that a turn stays inside its cell.
//
//   ./run_tests.sh explore

#include "Explore.h"
#include "Path.h"

#include <cmath>
#include <cstdio>

static int failures = 0;
static void expect(const char *what, bool ok) {
    if (!ok) { std::printf("FAIL: %s\n", what); failures++; }
}
static void expect_near(const char *what, float got, float want, float tol) {
    if (!(std::fabs(got - want) <= tol)) {
        std::printf("FAIL: %s -- got %.3f, want %.3f (tol %.3f)\n", what, got, want, tol);
        failures++;
    }
}
static void prep(SpeedPath &p, float yaw0 = 0.0f) {
    p.clear(0.0f, 0.0f, yaw0);
    p.set_clothoid_len(EX_TURN_CLOTHOID_MM);
}

// Path frame: x along the entry heading, y to the left. The mouse enters a
// cell at its boundary (x = 0); the cell spans x 0..180, y -90..90.
static void test_in_cell_turn_geometry() {
    float lead = ex_turn_lead_mm();
    expect("lead is positive", lead > 0.0f);
    expect("lead leaves room past the decision", lead > EX_DECIDE_MM + 10.0f);
    SpeedPath p; prep(p);
    expect("lead", p.add_straight(lead));
    expect("arc", p.add_arc(EX_TURN_R_MM, 90.0f));
    expect("exit", p.add_straight(lead));
    // The turn ends on the cell's centre line heading left, and the exit
    // straight reaches the left boundary exactly.
    PathPose e = p.pose(p.length_mm());
    expect_near("exit x = cell centre", e.x_mm, 0.5f * EX_CELL_MM, 0.05f);
    expect_near("exit y = left boundary", e.y_mm, 0.5f * EX_CELL_MM, 0.05f);
    expect_near("exit heading 90", e.yaw_deg, 90.0f, 0.05f);
    // Nothing leaves the cell; clearance to the inner post (entry-left
    // corner at (0, 90)) and to the outer wall faces (6 mm inside the grid).
    float min_post = 1e9f, min_wall = 1e9f;
    for (float s = 0.0f; s <= p.length_mm(); s += 1.0f) {
        PathPose q = p.pose(s);
        expect("inside x", q.x_mm >= -0.05f && q.x_mm <= EX_CELL_MM + 0.05f);
        expect("inside y", q.y_mm >= -90.05f && q.y_mm <= 90.05f);
        float dp = std::hypot(q.x_mm, q.y_mm - 90.0f);
        if (dp < min_post) min_post = dp;
        float dw = EX_CELL_MM - 6.0f - q.x_mm;
        float dr = q.y_mm + 90.0f - 6.0f;
        if (dw < min_wall) min_wall = dw;
        if (dr < min_wall) min_wall = dr;
    }
    expect("inner post clearance", min_post >= 41.0f);
    expect("outer wall clearance", min_wall >= 35.0f);
    std::printf("in-cell R%.0f/L%.0f turn: lead %.1f, post %.1f mm, walls %.1f mm, "
                "length %.1f mm, %d segments\n",
                (double)EX_TURN_R_MM, (double)EX_TURN_CLOTHOID_MM, (double)lead,
                (double)min_post, (double)min_wall, (double)p.length_mm(),
                p.seg_count());
}

// The extension arithmetic the cell records use: after each kind of
// extension the path ends at the NEXT cell's turn point, `lead` past its
// entry boundary, on that cell's centre line.
static void test_extension_lands_on_turn_points() {
    float lead = ex_turn_lead_mm();
    SpeedPath p; prep(p);
    float starty = 51.0f;
    float to_b = EX_CELL_MM - starty;
    expect("launch", p.add_straight(to_b + lead));
    float s_p = p.length_mm();
    // Straight through (0,1): next turn point 180 on.
    expect("straight", p.add_straight(EX_CELL_MM));
    expect_near("(0,2) turn point", p.length_mm(), s_p + EX_CELL_MM, 1e-3f);
    PathPose a = p.pose(p.length_mm());
    expect_near("still on the centre line", a.y_mm, 0.0f, 1e-3f);
    expect_near("x = (0,2) boundary + lead", a.x_mm, to_b + EX_CELL_MM + lead, 1e-3f);
    // Right turn in (0,2): the primitive plus 2 x lead lands at (1,2)'s turn
    // point, `lead` past the right boundary, heading east on (0,2)'s centre
    // line.
    expect("arc R", p.add_arc(EX_TURN_R_MM, -90.0f));
    expect("exit+lead", p.add_straight(2.0f * lead));
    PathPose b = p.pose(p.length_mm());
    expect_near("heading east", b.yaw_deg, -90.0f, 0.05f);
    expect_near("x on (0,2)'s centre line", b.x_mm, to_b + 1.5f * EX_CELL_MM, 0.05f);
    expect_near("y = lead past the right boundary", b.y_mm, -(0.5f * EX_CELL_MM + lead), 0.05f);
    // Segment budget: straights merge; a clothoid turn is 3 + 1 segments.
    expect("segments", p.seg_count() == 5);
    expect("corners", p.corner_count() == 1);
}

// The stop: (90 - lead) on from the turn point is the cell centre; and
// from a centre, (90 + lead) reaches the next cell's turn point.
static void test_stop_lands_on_centre() {
    float lead = ex_turn_lead_mm();
    SpeedPath p; prep(p);
    expect("lead", p.add_straight(lead));
    expect("to centre", p.add_straight(0.5f * EX_CELL_MM - lead));
    expect_near("centre", p.length_mm(), 0.5f * EX_CELL_MM, 1e-3f);
    SpeedPath q; prep(q);
    expect("from centre", q.add_straight(0.5f * EX_CELL_MM + lead));
    expect_near("next turn point from a centre", q.length_mm(), 90.0f + lead, 1e-3f);
}

// The planner at search speed: with the path ending at the turn point the
// mouse brakes only over v^2/2a before it (13 mm at 200 mm/s, 1500), which
// is after the decision point; extend it and the cap is gone, and the arc
// is flown at full speed.
static void test_planner_brakes_late_and_extension_lifts_it() {
    float lead = ex_turn_lead_mm();
    SpeedPath p; prep(p);
    p.set_limits(200.0f, EX_ACC_MM_S2, EX_ALAT_MM_S2);
    float end = 129.0f + lead;   // (0,1)'s turn point from starty 51
    expect("lead", p.add_straight(end));
    float s_brake = 0.0f;
    for (float s = 0.0f; s < end; s += 0.5f) {
        SpeedPath::Sample sm = p.sample(s, 200.0f, 0.002f);
        if (sm.v_mm_s < 199.0f) { s_brake = s; break; }
    }
    float s_decide = 129.0f + EX_DECIDE_MM;
    expect("braking begins after the decision point", s_brake >= s_decide);
    expect("straight extension", p.add_straight(EX_CELL_MM));
    SpeedPath::Sample sm = p.sample(end - 4.0f, 200.0f, 0.002f);
    expect_near("cap lifted by the extension", sm.v_mm_s, 200.0f, 0.5f);
    expect("arc", p.add_arc(EX_TURN_R_MM, 90.0f));
    SpeedPath::Sample sa = p.sample(p.length_mm() - 30.0f, 200.0f, 0.002f);
    expect("200 through the arc", sa.v_mm_s >= 199.0f);
    std::printf("planner at 200: braking from %.1f, decision at %.1f, turn point %.1f; "
                "arc at %.0f mm/s\n", (double)s_brake, (double)s_decide, (double)end,
                (double)sa.v_mm_s);
}

// The hard front guard's "where am I in this cell" arithmetic
// (Explore.cpp front_hard_stop): the path's pose at s against its pose at
// the cell's entry boundary, rotated into the entry heading, gives (along
// past the centre, lateral left); the distance past the centre along the
// heading the mouse will leave by is what path_from_centre gets. Checked
// mid-way through a LEFT turn entered heading east (yaw -90): the arc
// leaves the entry axis to the left, so "past the centre along the exit
// heading" is the lateral offset. 2026-09-26: the (5,5) stop at 66 deg
// stood ~13 mm short of the centre on the entry axis and ~33 mm left.
static void test_hard_stop_pose_arithmetic() {
    SpeedPath p; prep(p, -90.0f);              // entering heading east
    float lead = ex_turn_lead_mm();
    p.add_straight(lead);                       // boundary -> turn point
    p.add_arc(EX_TURN_R_MM, 90.0f);             // left turn
    p.add_straight(2.0f * lead);
    float s_b = 0.0f;
    // Walk the arc to 66 deg of heading change.
    float s = lead, yaw0 = -90.0f, s66 = -1.0f;
    for (; s < p.length_mm(); s += 0.5f) {
        float dy = p.pose(s).yaw_deg - yaw0;
        while (dy > 180) dy -= 360; while (dy < -180) dy += 360;
        if (dy >= 66.0f) { s66 = s; break; }
    }
    expect("reached 66 deg inside the turn", s66 > 0);
    PathPose pn = p.pose(s66), pb = p.pose(s_b);
    float thb = pb.yaw_deg * 3.14159265f / 180.0f;
    float dx = pn.x_mm - pb.x_mm, dy = pn.y_mm - pb.y_mm;
    float along = dx * std::cos(thb) + dy * std::sin(thb) - 0.5f * EX_CELL_MM;
    float lateral = -dx * std::sin(thb) + dy * std::cos(thb);
    std::printf("hard-stop pose at 66 deg: along %+.1f lateral %+.1f (s %.1f)\n", along, lateral, s66);
    // Plain R55 arc from the turn point: 55 sin66 = 50 along, 55 (1 - cos66) = 33 left;
    // the clothoids move the numbers a little.
    // (the 16 mm entry clothoid runs straighter for longer: +8 along vs the plain arc)
    expect_near("along past the centre", along, lead + 55.0f * std::sin(66.0f * 3.14159265f / 180.0f) - 90.0f, 10.0f);
    expect_near("lateral, left of the entry axis", lateral, 55.0f * (1.0f - std::cos(66.0f * 3.14159265f / 180.0f)), 6.0f);
    expect("left of the axis in a left turn", lateral > 0.0f);
    // Exit heading is LEFT of entry: past the centre along it = lateral.
    float past = lateral;
    expect("past the centre along the exit heading is the lateral offset", past > 20.0f && past < 45.0f);
}

int main() {
    test_hard_stop_pose_arithmetic();
    test_in_cell_turn_geometry();
    test_extension_lands_on_turn_points();
    test_stop_lands_on_centre();
    test_planner_brakes_late_and_extension_lifts_it();
    if (failures) { std::printf("%d failure(s)\n", failures); return 1; }
    std::printf("test_explore: all passed\n");
    return 0;
}
