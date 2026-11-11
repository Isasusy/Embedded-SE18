
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
