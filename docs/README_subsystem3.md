
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

GP28 is now ADC channel 2 (like the line sensors), sampled every 1 ms and turned into black/white with
`BAR_ADC_THRESH` +- `BAR_ADC_HYST`. A digital read never saw the black bars: an analog IR sensor gives only ~1.7 V over
black, below the RP2040's logic-high level. The 2 Hz `[S3] adc` line now shows `barcode=NNNN (black|white)`.
The ADC is shared by line_task and barcode_task under a spin lock (they may run on different cores).

## Barcode bars are not junctions (11 Nov)

If a barcode's bars run under the line sensors too, they look like a junction. line_task now drops a JUNCTION while the
barcode decoder is mid-code or within `BAR_JUNCTION_BLANK_MS` of its last stripe, and prints why.
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
first bar stretch the end-of-code wait or drop the read. The logged run decodes as "*F*" (forwards and backwards); see
the test in testing/subsystem3/pc/test_s3.c.

## No fixed barcode size (11 Nov)

Nothing in the decoder depends on the printed size or the speed: bars are thick/thin relative to each other (by width,
or by darkness when the sensor blurs), each character's wide elements are found relative to that character, and the
end of a code is "white for longer than the widest bar seen" (an early finish as soon as whole characters decode, else
4x). The only absolute limits are a 2 s dark stripe (= the line) and 64 stripes (up to 4 letters between the '*'s).
Tested: narrow 1.5-15 mm, wide 2-3 x narrow, 50-200 mm/s, both directions, plus the two real blurred runs.
What cannot be fixed in software: the sensor must pass over the bars, and very small bars (narrow < ~2.5 mm with this
sensor's blur) cannot be separated.
