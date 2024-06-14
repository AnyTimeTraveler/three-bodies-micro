# Three Bodies rewritten to run on an ESP32-C6

This is a simple three body simulation, built over a lunch break to try
out [macroquad](https://docs.rs/macroquad/latest/macroquad/index.html), rewritten to run on [embedded-graphics](https://crates.io/crates/embedded-graphics) in an evening.

An online version can be found [here](https://three-bodies.sulami.xyz/) or in this video:

https://github.com/sulami/three-bodies/assets/1843193/d8a41847-a475-46c8-8eb9-396d64411175

This rewritten version can be seen here:

https://github.com/AnyTimeTraveler/three-bodies-micro/assets/19378309/790d6573-3483-430e-a72b-ea2eb87a70d4

_Note:_ If you are using a non-standard keyboard layout, the controls do not take that into account.

## Building

For use with esp-flash, run

```sh
cargo run --release
```
