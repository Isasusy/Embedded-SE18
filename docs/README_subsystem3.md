
## Sensor geometry (from the project write-up, 6 Nov)

The two IR sensors sit just **outside one black line**, one on each side. Both white = on course, so the line
follower drives straight and steers back only when a sensor sees black (`LINE_STRADDLE 1`). Because both-white is
normal, **LINE_LOST is never raised** in this mode; `LINE_FOUND` is raised when a sensor first reaches the line after a
quiet stretch (the obstacle bypass uses it to know it is back on the line). If the car steers the wrong way when one
sensor sees black, change `LINE_STEER_TOWARD_BLACK` to 0 (that is the layout for sensors outside two lane edges).
Both sensors black for 30 ms = a perpendicular line = **JUNCTION**.

## Curves (11 Nov)

Steering gain doubled (`LINE_STEER_GAIN` 240) and `MC_STEER_MAX` raised to 200 mm/s. As soon as one sensor touches the line
(`LINE_STEER_ENGAGE`, 25 % black) the car steers at least `LINE_STEER_MIN`, and when the sensor leaves the line the turn is
held and fades out over `LINE_STEER_HOLD_MS`, so a curve is followed instead of going straight on. Tune: if it weaves on
straights lower `LINE_STEER_MIN` or `LINE_STEER_HOLD_MS`; if it still runs wide on curves raise them.

## Barcode: Code 39 (11 Nov)

The decoder reads Code 39 (e.g. `*A*`) in either direction: each character is 9 elements, the 3 widest are wide, so the
speed does not matter. A = left, B = right, C = straight, D = U-turn. EVERY read is printed on the console:
`[S3] >> BARCODE "A" -> LEFT (pattern ..., narrow N ms)` or `[S3] barcode NOT decoded: N elements, pattern ...`.
The pattern shows what the sensor saw (W wide, n narrow, a space between characters). If the pattern starts on white
or looks inverted, flip `BARCODE_BLACK_IS_HIGH`. The last code is also kept in `s3_last_barcode` for the telemetry.

## Barcode sensor read through the ADC (11 Nov)

GP28 is now ADC channel 2 (like the line sensors), sampled every 1 ms and turned into black/white (since superseded:
see the peak detector below; `BAR_ADC_THRESH` now only classifies the very first sample). A digital read never saw the black bars: an analog IR sensor gives only ~1.7 V over
black, below the RP2040's logic-high level. The 2 Hz `[S3] adc` line now shows `barcode=NNNN (black|white)`.
The ADC is shared by line_task and barcode_task under a spin lock (they may run on different cores).

## Junctions and the barcode read (11 Nov, v23)

The course barcode sits beside the line with a 20 mm gap, so the line sensors never see its bars: both line sensors
black is a junction. (v19-v22 ignored a junction while a barcode read was still open; on 11 Nov 19:24 that made the car
drive through the junction it should have stopped at.) Now:
- both line sensors above `LINE_LIFTED_ADC` (nothing under them) = car lifted, not a junction;
- a junction while a barcode read is still open ends that read there (`bd_finish`) and decodes what was read, dropping
  a trailing white and, if needed, one last bar that is the junction's own line;
- S5 takes a barcode command that arrives during the 1 s junction pause and does it at that junction.
The 2 Hz `[S3] adc` line ends with `last read: X` (`none yet` / `FAILED`) so the result is visible even if the
moment it was printed was missed.
`S3_BARCODE_TRACE 1` prints every stripe the barcode sensor sees (`[S3] bar: BLACK 42 ms`).

## Course barcode and the peak detector (11 Nov)

The course's barcode (Barcode & Line Specification) is Code 39 `*A*`-style, 141 mm long, beside the 18 mm line with a
20 mm gap: narrow elements are 3 mm, wide 9 mm. At 90-110 mm/s a 3 mm element lasts ~30 ms, i.e. ~30 samples at
1 kHz, so sampling speed is not the limit. The IR spot is: it blurs 3 mm elements so they never reach full black or
white. GP28 is now turned into stripes by a peak detector (`BAR_SWING`), not a fixed threshold. Simulation
(testing/subsystem3/pc/barcode_sim.c): with a spot of sigma 2 mm the code decodes at 100 and 250 mm/s, a fixed
threshold fails; with sigma 2.5-3 mm nothing can separate the 3 mm gaps, so the sensor must sit low (2-5 mm above
the paper) and over the bars (about 30-60 mm to the side of the line centre).

## Decoding a blurred barcode (11 Nov, from the 18:48 run)

At BAR_SWING 70 the sensor reported all 29 stripes of the course barcode, but blurred: thin bars only reach ~300 (thick
~2400) and look short, so the spaces beside them look long. The decoder now (1) calls a bar thin/thick by how DARK it
gets, (2) finds the wide space of each character from the distances between bar CENTRES, and (3) does not let a slow
first bar stretch the end-of-code wait or drop the read. The logged run decodes as "*F*" (forwards and backwards). Note: testing/subsystem3/pc/test_s3.c in this
repository still targets the older decoder API and must be updated before it builds.

## No fixed barcode size (11 Nov)

Nothing in the decoder depends on the printed size or the speed: bars are thick/thin relative to each other (by width,
or by darkness when the sensor blurs), each character's wide elements are found relative to that character, and the
end of a code is "white for longer than the widest bar seen" (an early finish as soon as whole characters decode, else
4x). The only absolute limits are a 2 s dark stripe (= the line) and 64 stripes (up to 4 letters between the '*'s).
Tested: narrow 1.5-15 mm, wide 2-3 x narrow, 50-200 mm/s, both directions, plus the two real blurred runs.
What cannot be fixed in software: the sensor must pass over the bars, and very small bars (narrow < ~2.5 mm with this
sensor's blur) cannot be separated.

## Thick bars by length, and thin / medium / thick bars (12 Nov)

A bench read of `*A*` failed although all 29 stripes were seen: the LAST bar lasted 118 ms (thin bars 99-140 ms, thick
380-519 ms) but got as dark (2842) as a thick bar, so the darkness rule called it thick and the last `*` became invalid.
The decoder now finds the thick bars in this order, and keeps the first that makes the whole code valid:
1. **bar length**: the bar durations are sorted and split where they jump the most (`BAR_LEN_SPLIT_RATIO`, 1.5x);
2. **bar length, 2nd split**: the second biggest jump. If the course's bars come in three sizes (thin, medium,
   thick), the two splits are the two meanings of "medium" (counted wide, or counted narrow), so one of them fits;
3. **bar darkness** (the previous rule);
4. **element widths**: the 3 longest elements of each character are the wide ones (no blur handling).
The first and last bar are always set thin: `*` starts and ends with a thin bar in either reading direction, and those
two stripes are the least reliable (slow rise into the first bar, the junction line after the last).
A valid read now says how it was decoded: `[S3] >> BARCODE "A" -> LEFT (... by bar length)`.
A failed read prints every stripe (`S3_BARCODE_DUMP 1`): `[S3]   stripes 0-9: B120:2000 s800:190 ...` = bar 120 ms
with peak 2000, space 800 ms ... one line per Code 39 character. On the day, read the real barcode's sizes off that.
Checked: compiles with the Pico toolchain (-Wall -Wextra, no warnings); a Python port of the decoder replayed on the
logged stripes gives FAILED (old) and "A" (new), both directions; 800 synthetic reads (A-D, both directions, +-15 %
speed wobble, medium bars counted wide or narrow) all decode. Not yet run on the car.
If the course barcode turns out NOT to be Code 39 (e.g. Code 128, which has 4 bar sizes), this decoder cannot read
it: send the `stripes` dump and the decoder can be adapted.

## Dark fragments and a damaged '*' (12 Nov, second run)

The car stopped at the junction but drove straight on: the `*A*` read before it had failed. Two causes:
- **Dark fragments.** The thick bars read 3500-4000 and flickered by more than `BAR_SWING`, so one bar arrived as up
  to 13 stripes (63 instead of 29). The "spaces" between the fragments never got lighter than ~3000 (real spaces ~200).
  Now every space whose lightest reading stays in the top `BAR_DIP_LEVEL_PCT` (75 %) between white (the lightest space)
  and dark (the upper quartile of the bar peaks) is merged into the bar, before decoding, whatever the count.
- **A damaged first `*`.** Even merged, the first character was garbled (the car was steering onto the barcode).
  With `BAR_ALLOW_BAD_GUARD 1` a code is accepted when ONE `*` is unreadable but the other `*` and the letter are
  valid; the log then says `(one * damaged)`. A fully valid read by any method always wins. The early finish (before
  the quiet zone) never uses this, because there the code may still go on.
Checked with a Python copy of the decoder on the logged stripes: run 1 -> A (clean), run 2 -> A (one * damaged), both
directions; the 43-stripe floor-noise read stays rejected; 400 synthetic codes with split bars (half with a damaged
first `*`) all decode. Trade-off: on random junk that happens to have exactly 29 or 39 stripes, 95 in 20000 (0.5 %)
now come out as a command, against 3 in 20000 with `BAR_ALLOW_BAD_GUARD 0`. Set it to 0 if a wrong turn is worse than a
missed one.
