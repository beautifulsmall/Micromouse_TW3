// Inside-wall EDGE gate: anchor the fast run's along-track position on the
// end of the wall the corner turns around.
//
// THE PROBLEM. At speed the wheels slip, so path position s -- which is
// fused odometry -- drifts. Every corner is then entered at a position the
// mouse only believes. The wall the corner turns AROUND ends exactly at the
// corner cell's near boundary, by construction of the maze, and that end is
// a landmark whose position is known to the millimetre from the map. Seeing
// it says where the mouse actually is, along-track, with no pose estimator
// in the loop.
//
// WHAT AN EDGE LOOKS LIKE (bert's spec, 2026-07-27). Not a drop to nothing.
// Past the wall's end the side beam usually lands on the NEXT wall down the
// corridor -- a VALID reading 90-180 mm farther out. The signature is the
// STEP, and a valid far reading is strong evidence (2 samples confirm). A
// non-return is weak evidence: 255 means INVALID, not absence (both over-
// and under-range read 255), so it takes 3 samples. Position is taken at
// the FIRST evidence either way, not at confirmation.
//
// SILENT OR CERTAIN. A correction outside the plausibility window is not
// applied and not half-applied -- it is dropped, and the corner flies
// ungated. V1 learned this at cost: a -53 mm edge "correction" jolted an
// approach into a +12 mm entry transient and a crash (2026-07-26). An
// ungated corner is a corner flown on odometry, which is what the mouse did
// before gates existed; a wrong correction is worse than no correction.
//
// THE OFF-CENTRE GUARD. Drifted far enough off the corridor centreline, the
// inside beam works near its range limit and presence flickers -- false
// drops gated one corner at -52 mm and another at +35 mm (2026-07-16/17).
// The lateral offset is judged at the LAST sample where the wall was still
// present, not at the drop, because by the drop the lateral fit has often
// gone invalid already. An untrusted corner goes ungated; it does NOT
// consume the gate, so the debounce restarts and a later clean approach can
// still anchor it.
//
// BEAM LEAD. The side beam's hit point rides ahead of the mouse (mount
// offset plus beam angle to a corridor wall), so the drop is observed
// WALL_GATE_BEAM_AHEAD_MM before the mouse reaches the wall's end. The
// caller folds that into expect_s_mm; this module only compares where the
// drop was seen against where it was predicted.
//
// Pure logic -- no HAL, no floats beyond arithmetic. Host-tested in
// test/test_wall_gate.cpp.

#pragma once

#include <stdint.h>

// Distance the side beam's hit point leads the mouse on a corridor wall.
//
// 72 was a V1 DESK CALCULATION -- 40 mm of mount offset plus the beam angle
// to a wall 82.5 mm away -- and it was never checked against a wall the mouse
// had actually driven past. It is wrong by about 25 mm, and that single error
// is the whole of what runs 130-132 were suffering from.
//
// THREE INDEPENDENT MEASUREMENTS, none of which knew about the others:
//
//   run 122   arc-start geometry, physical arc start vs nominal  ~51 mm
//   runs 130-132  drop -> first in-arc sample, 81 gated corners  ~48 mm
//   runs 130-132  censored-normal fit to gate_corr, n=147        ~39 mm
//
// 45 sits inside that bracket and deliberately under-corrects: a residual
// negative bias still gates (the window is wide enough now), whereas
// over-correcting flips the sign of every correction in the log and throws
// away the reference frame that made these three numbers comparable.
//
// WHY IT MATTERED SO MUCH. The error shows up as a systematic gate_corr of
// -24.5 mm, every corner, at both speeds. Where the gate fires it measures
// that offset and removes it, and the corner exits within 2 mm at EVERY
// radius from 125 down to 60. Where |err| drifts past the window the gate
// refuses, the offset stands, and the corner cuts 18 mm to the inside --
// also at every radius. Two flat populations:
//
//              gate LOCKED    gate SKIPPED    skip rate
//     R125       +1.1 mm        -16.7 mm        25 %
//     R100       -0.3 mm        -18.2 mm        40 %
//     R80        -0.3 mm        -19.8 mm        61 %
//     R60        +1.6 mm        -17.2 mm        87 %
//
// Neither column depends on radius. Only the MIX does -- which is why the
// pooled average looked like oversteer growing as the turns tightened, and
// why chasing it with a steering-gain correction would have been fixing a
// statistic rather than a fault.
//
// It is a FIXED offset, not a scale error: corner 0 has a short approach and
// shows the same -20.0 mm ungated exit as the full-845 mm corners (-18.0,
// -21.0, -15.7), and the same gate_corr (-26.3 vs -23.8). The odometry is
// sound. Re-measure with 'g' if the optics move.
// 45 -> 58: THE LEAD IS THE SPOT, AND THE SPOT IS AHEAD OF THE NOSE.
//
// bert, 2026-08-30: "a distance of 45mm would put them level with the front
// of the mouse, the laser sensors point ahead." The chassis is 90 mm with a
// CENTRAL axle, so 45 mm from the datum IS the nose -- 45 was never a
// measurement, it was a fitted residual that happened to land on the body
// length. The side lasers sit 40 mm forward of the datum and look ahead, and
// bert's ruler puts their spots at 63 / 65 mm ahead of centre on a centred
// mouse. Call the lead 64.
//
// The gate expects the drop at datum - 90 - BEAM_AHEAD, while the wall+post
// surface really ends at datum - 84 (post FAR face) and the spot leads the
// body by `lead`, so the drop truly happens at datum - 84 - lead:
//
//     err = expect - drop = lead - BEAM_AHEAD - 6
//         = 64 - 45 - 6 = +13 mm      <- the side gate reads 13 mm high
//
// Which is exactly the +13.5 (sd 1.7, 9 paired corners) that the FRONT gate
// measured against this one, and it says the bias is HERE, not there. Two
// further checks agree: at the packet where the side beam loses its wall the
// front beam should read (84 + 84) + lead = 232 and it reads 235 over 11
// approaches (front honest to 3 mm); and BEAM_AHEAD = lead - 6 = 58 zeroes
// the residual by construction.
//
// Behaviour is unchanged by this: the glearn table absorbs an expectation
// shift within 2-3 laps (the same mechanism that neutralised goff), so the
// arcs land where they did and atrim stays 30. What changes is that the two
// gates now share an honest datum -- ftrim goes to 0, and a goff entry
// becomes pure rig geometry instead of rig geometry plus 13 mm of constant
// error, so it means something on a maze this mouse has never seen.
// GOFF_TABLE_GEOM_REV is bumped with this: the stored table is stale.
constexpr float WALL_GATE_BEAM_AHEAD_MM = 58.0f;

struct WallGateConfig {
    // Path s at which the drop is PREDICTED to be observed, i.e. the wall
    // end's path position minus WALL_GATE_BEAM_AHEAD_MM.
    float expect_s_mm = 0.0f;
    // Plausibility window. V1's silent-or-certain value was 35, chosen when
    // the bias it sat on top of was unknown. Runs 130-132 measured the
    // residual spread around the (now corrected) centre at sd 12.5 mm, so a
    // +/-35 window is only 2.8 sigma and still censors a few per cent of
    // corners -- and a censored corner is not a small error, it is the FULL
    // 18 mm offset plus whatever the latch adds on the next lap.
    //
    // 45 is 3.6 sigma. The cost is admitting an edge up to 45 mm wrong; the
    // benefit is that the tail stops being refused. That trade only became
    // the right one once the centre was fixed -- widening the window while
    // the distribution still sat at -33 would have been letting bad
    // corrections in rather than letting good ones through.
    float window_mm = 45.0f;
    // A raw beam reading at or below this counts as the wall being there.
    uint8_t present_mm = 165;
    // Wall acquisition, DISTANCE-BASED (2026-08-29). The old rule was 12
    // PACKETS, which means 8 mm of wall at v500 and 21 mm at v1300 -- so a
    // 12 mm post that run 148 proved acquirable at gathering speed became
    // invisible at race speed, and the NO_EDGE rate rose with rv for no
    // physical reason. A packet count is a speed-dependent ruler. The wall
    // must now SPAN present_span_mm of travel AND deliver at least
    // present_n clustered packets (bert: "4 clustered hits should be
    // enough" -- the count floor keeps a single glitch pair from
    // impersonating a surface at very high speed).
    int present_n = 4;
    float present_span_mm = 8.0f;
    // A valid reading this much farther than the tracked wall is a STEP.
    float step_mm = 60.0f;
    int step_n = 2;    // step confirmations (strong evidence)
    int absent_n = 3;  // non-return confirmations (weak evidence)
    // Drop confirmation must ALSO span this much travel (2026-08-29). The
    // wall-post construction leaves a ~2 mm gap between the wall's end and
    // the 12 mm post; at slow (braking) approaches that gap is ~3 packets,
    // which met absent_n and fired the gate at the GAP -- 14-15 mm before
    // the post's far edge, which is the -15 mm fingerprint runs 203-207
    // measured at the braking corners. (Fast approaches cross the gap in
    // one packet and survived -- the bug was speed-dependent, which is what
    // disguised it as slip.) Requiring the evidence to persist 4 mm of
    // travel makes the gap unbelievable at ANY speed: the post's return
    // resumes tracking and the same wall carries on to its true end. The
    // position is still stamped at FIRST evidence, so confirmation distance
    // adds no bias. A real opening is a cell wide; 4 mm costs nothing.
    float drop_confirm_mm = 4.0f;
    // LATERAL COMPENSATION of the drop (2026-08-29, bert's bench numbers).
    // The beam's along-track lead depends on how far the wall is: centred,
    // the spot rides 63-65 mm ahead; ride closer to the inside wall and the
    // lead shortens at ~1 mm per mm of lateral (the fitted 39-51 mm leads
    // vs the centred 64 imply a ~45 deg beam). So a mouse offset TOWARD the
    // inside wall sees the drop LATER than the centred model expects, by
    // klat x offset. The caller passes lat already signed toward-the-inside
    // (positive = closer to the wall being watched); the drop is corrected
    // by that much before the error is computed. 0 = no compensation (the
    // pre-2026-08-29 behaviour); set from a bench measurement of the lead
    // at centre vs at a known offset, via the registry (`gklat`).
    float klat = 0.0f;
    // Off-centre guard: |lateral| above this and the corner goes ungated.
    //
    // 13 was too tight, and the cost of refusing turned out to be far larger
    // than the error being refused. Run 127, 83 corners:
    //
    //     gate LOCKED   48 corners (58 %)   mean |exit_lat|  7.7 mm
    //     gate SKIPPED  33 corners (35 % on lateral)         17.2 mm
    //
    // Better than two to one, and the skip is a LATCH: no correction means
    // the arc is placed on odometry alone, so the exit is wide, so the servo
    // has more to drain over the following straight and overshoots further,
    // so the next entry is further off and skips again. Four sessions have
    // now ended inside that loop -- run 127's last corner entered 13.6 mm
    // off, skipped, and exited 66 mm into the outer wall.
    //
    // The reason for the limit is real: a mouse off centre sees the wall end
    // at a different point along the beam, so the drop is biased by roughly
    // lat * (beam_ahead / beam_side) ~ 0.6 * lat. At 25 mm that is ~15 mm of
    // along-track error. But the alternative is not zero error, it is NO
    // CORRECTION AT ALL, and that error is unbounded -- it compounds lap over
    // lap. A bounded bias beats an unbounded one.
    //
    // The principled fix is to COMPENSATE rather than refuse, since the gate
    // already knows the lateral. That wants the beam geometry measured first
    // (the effective look-ahead came out ~51 mm in run 122, against the 72
    // assumed), so it waits for a bench measurement rather than a guess.
    //
    // 25 -> 35, run 140. The session's one failure was refused by 0.7 mm.
    // Loop 17 corner 0 entered at -25.7 (every other loop in the run launched
    // between -1.2 and -8.8), hit SKIPPED_LAT, flew ungated, exited -23.3,
    // drifted to -40.5 and tripped the guard. The lap that needed the gate
    // most was the only lap the gate refused, which is the latch described
    // above arriving in one step instead of over four.
    //
    // 35 is sized on the window, not on comfort. The refusal exists because
    // the drop is biased by lat * (beam_ahead / corridor_half) = lat * 45/84
    // = 0.54 * lat. At 35 that is 19 mm of along-track bias, which still sits
    // inside the +/-45 window -- so a lateral bad enough to matter gets
    // caught by the WINDOW test on its merits rather than pre-emptively by a
    // proxy. Past ~45 mm the two tests would agree anyway and the lateral
    // limit stops doing independent work.
    //
    // The compensation is now buildable and this is the interim: both terms
    // have been measured (WALL_GATE_BEAM_AHEAD_MM = 45 from three independent
    // fits, corridor half-width 84 by construction), so 0.54 is no longer the
    // guess it was when this comment was written. Subtracting 0.54*lat from
    // the drop would let the limit go back down. Not done here because it
    // changes every gated corner, and run 140 only justifies changing the
    // refused ones.
    float lat_max_mm = 35.0f;
    // OBSERVE WIDE, DECIDE NARROW. The caller arms the gate a fixed distance
    // before the expected drop, and that one distance has been doing two
    // jobs: giving the presence debounce runway, and bounding how far from
    // the prediction an edge may be believed. They want different sizes --
    // the second should be tight, because a wall end 200 mm from where the
    // map says it is is not that wall end; the first wants to be generous,
    // and the feature observer wants to be more generous still, because the
    // whole point of counting features is to see ALL of them.
    //
    // So the gate now observes from the moment it is armed and only DECIDES
    // once s reaches this. Default 0 means "decide from arming", which is
    // the behaviour every run up to 146 had.
    float live_from_s_mm = 0.0f;
};

class WallGate {
   public:
    enum Result : uint8_t {
        NONE = 0,      // nothing to report this sample
        CORRECTION,    // an edge was found and *corr_mm is the error
        SKIPPED_LAT,   // edge found but the mouse was too far off centre
        SKIPPED_WINDOW // edge found but the implied correction is implausible
    };

    void arm(const WallGateConfig &cfg);
    void disarm() { armed_ = false; }
    bool armed() const { return armed_; }
    // Set once a result has been produced; the gate fires at most once per
    // arming for a CORRECTION, but a SKIP leaves it armed to try again.
    bool fired() const { return fired_; }

    // Feed one fresh sensor packet. `inside_raw_mm` is the RAW beam range on
    // the inside of the turn (the driver's left_mm for a left corner,
    // right_mm for a right one); 255 and 0 are invalid. Returns what
    // happened; on CORRECTION, *corr_mm is the along-track error
    // (positive = the mouse is FARTHER along than odometry believed) and is
    // what to hand to control_path_correct().
    Result update(float s_mm, uint8_t inside_raw_mm, bool lat_valid,
                  float lat_mm, float *corr_mm);

    // Diagnostics for the run report.
    float drop_s_mm() const { return drop_s_; }
    float last_lat_mm() const { return lat_at_wall_; }
    int present_run() const { return present_run_; }

    // ---- THE FEATURE OBSERVER ----------------------------------------------
    //
    // Purely passive. It changes nothing the gate decides; it writes down
    // what the inside beam DID, so the landmark logic a real maze needs can
    // be designed from measurements instead of from a desk.
    //
    // WHAT IT IS FOR. This gate anchors on the end of ONE wall and fires
    // once, treating everything else on the approach as noise to be refused.
    // bert's maze does not look like that (2026-08-22): "there may be many
    // walls and gaps before a turn, the mouse should count these and gate
    // the turn from the last good edge." That inverts the problem. After a
    // search the mouse HAS the map, so it knows exactly how many wall ends
    // and openings the approach contains and where each one is. They are not
    // decoys; they are a SEQUENCE, and a sequence is a stronger landmark
    // than any single edge in it -- while the LAST edge before the turn is
    // the one with the least odometry drift after it.
    //
    // None of that can be written until we know what the beam actually
    // produces. So the observer records RUNS: every unbroken stretch where
    // the inside beam sees something near, with where it started, where it
    // ended, how many packets it lasted and how close it got. A wall is a
    // long run. A post is a short one -- 12 mm of it, which is a handful of
    // packets at flight speed and is exactly why `present_n = 12` cannot see
    // one. A gap is the space between two runs.
    //
    // On an intact corner this measures a wall of known length, which
    // calibrates packets-per-millimetre for free; on a stripped corner it
    // measures a bare post. Same instrument, built-in control.
    struct Run {
        float s0_mm;    // s at the first sample of the run
        float s1_mm;    // s at the last
        uint8_t min_mm; // closest reading in it
        uint8_t n;      // packets, saturating at 255
    };
    static constexpr int MAX_RUNS = 5;

    // WHAT RUN 148 MEASURED, and why this bridge exists.
    //
    // Two inner walls out, so two of the four approaches saw a bare post.
    // 26 post approaches:
    //
    //     post   30.8 +/- 5.1 packets   22.1 +/- 4.0 mm   min range 87.3 +/- 2.2
    //     wall      199 packets            151 mm         min range 86
    //
    // Two things fall out. A post is 2.5x LONGER than present_n = 12, not
    // shorter -- so the step detector already acquires one and drops cleanly
    // off its far end, and run 148 gated 52 of 52 with the stripped corners
    // exiting -6.2 and -1.0 mm against -0.5 for the intact ones. No post
    // detector is needed. And post against wall is 22 mm against 151 mm with
    // nothing in between, so a feature can be CLASSIFIED by its length alone,
    // which is what lets an observed sequence be matched to a map.
    //
    // The raw stream splits a post into two or three pieces: 39 precursor
    // blips of 2.5 packets at 107-140 mm, ahead of the solid run, separated
    // from it by 1.9 mm on average and 4 mm at worst. Those are dropouts
    // inside one feature, not features. Bridging them is what makes the COUNT
    // truthful -- and the count is the thing bert's gate is to be built on
    // ("count these and gate the turn from the last good edge"). At 8 mm,
    // every one of the 26 approaches collapses to exactly one run, with
    // twice the margin over the worst gap observed.
    static constexpr float BRIDGE_MM = 8.0f;

    int run_count() const { return runs_seen_; }        // total, may exceed MAX
    int runs_stored() const { return runs_n_; }
    const Run &run(int i) const { return runs_[i]; }

   private:
    void reset_runs();

    WallGateConfig cfg_;
    bool armed_ = false;
    bool fired_ = false;
    int present_run_ = 0;
    int absent_run_ = 0;
    int step_run_ = 0;
    float track_mm_ = 255.0f;   // closest reading seen on this wall
    float present_s0_ = 0.0f;   // s at the first packet of the present run
    float present_s1_ = 0.0f;   // s at the last tracking packet
    float drop_s_ = 0.0f;       // s at the FIRST evidence of the edge
    float lat_at_wall_ = 0.0f;  // lateral at the last wall-present sample

    // Feature observer. Kept strictly separate from present_run_ and friends
    // so it cannot perturb the gate: it reads the same samples and writes
    // only to itself.
    Run runs_[MAX_RUNS] = {};
    Run cur_ = {};
    bool in_run_ = false;
    int runs_n_ = 0;      // stored (<= MAX_RUNS)
    int runs_seen_ = 0;   // seen (may exceed MAX_RUNS)
    void observe(float s_mm, bool valid, float d);
};
