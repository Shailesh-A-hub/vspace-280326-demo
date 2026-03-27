<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

This is the final piece of the puzzle! Writing clear documentation ensures that when your chip comes back from the factory, anyone in the world can wire it up and use it.

You will put this text inside the docs/info.md file in your repository. Here is the perfect, professional documentation tailored specifically to your stopwatch logic.

How it works
The V-SPACE Demo Hardware Stopwatch is a 1-second interval counter that displays digits from 0 to 9 on a standard 7-segment display. The design is broken down into three main hardware modules:

+ The Clock Divider: The Tiny Tapeout board provides a default 10 MHz clock. Our Verilog code uses a 24-bit register to count exactly 9,999,999 clock cycles, generating a single 1 Hz pulse (one pulse per second).

+ The Digit Counter (BCD): Every time the 1 Hz pulse triggers, a 4-bit register increments. Once the counter reaches 9, it automatically wraps back around to 0.

+ The 7-Segment Decoder: Pure combinational logic continuously reads the 4-bit counter and outputs the correct 7-bit binary pattern to light up the corresponding segments (A through G) on an LED display.

The counting sequence is controlled by a hardware enable switch connected to ui_in[0]. When the switch is HIGH, the clock divider runs and the counter increments. When the switch is LOW, the clock divider pauses, freezing the current number on the display.

## How to test

To physically test this chip once manufactured (or when using the Tiny Tapeout Commander app):

+ Power & Clock: Ensure the Tiny Tapeout board is powered and the system clock is set to 10 MHz.

+ Reset: Press the system reset button (pulling rst_n LOW) to clear all internal registers. The 7-segment display should show 0.

+ Start Counting: Flip Input Switch 0 (ui_in[0]) to the HIGH (ON) position. The display will begin ticking up by one every second.

+ Pause Counting: Flip Input Switch 0 to the LOW (OFF) position. The display will freeze on the current digit.

+ Resume: Flip Input Switch 0 back HIGH to resume counting from the paused digit.

+ Hard Reset: At any time, pressing the reset button will immediately force the counter back to 0.

## External hardware

To view the output of this project, you will need:

+ Tiny Tapeout Demo Board (or equivalent carrier board).

+ 7-Segment Display PMOD connected to the dedicated output pins (uo_out[0:7]).

+ A simple DIP switch or push-button connected to input pin 0 (ui_in[0]) to act as the Start/Pause toggle.
