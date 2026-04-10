<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

The Low-Power SpO2 Ratio Engine is a specialized digital signal processor designed for pulse oximetry. It processes raw alternating Red and Infrared (IR) LED measurements from an external sensor to calculate blood oxygen saturation levels. 

To overcome the strict 1x1 tile area constraint of TinyTapeout, the architecture was heavily optimized:
* **Multiplier Removal:** It completely eliminates bulky combinational multipliers, utilizing an iterative shift-and-subtract state machine to calculate the `(AC_red * 8) / AC_ir` ratio, minimizing the footprint.
* **Synchronized Pipeline:** Operates via a 3-state FSM (`IDLE`, `COLLECT`, `DIV`) integrated with a robust SPI deserializer and clock-domain crossing synchronizers.
* **Component Extraction:** Automatically maintains a 2-sample moving average to act as DC baseline tracking, while extracting the AC peak values needed for the standard `R` ratio evaluation.

## How to test

To physically test this logic once manufactured:

* **SPI Data Input:** Drive sequential sampled values on `ui_in[0]` (SPI Data In) alongside the clock signal on `ui_in[1]`.
* **Channel Select:** Toggle `ui_in[2]` to denote whether the incoming data stream belongs to the Red channel (`0`) or the IR channel (`1`).
* **Reset**: Pulse the system reset button (pulling `rst_n` LOW) to initialize the FSM and synchronizers.
* **Ratio Output:** Read the 7-bit calculated SpO2 Ratio from `uo_out[0:6]`.
* **Data Valid:** Wait for `uo_out[7]` (Valid Flag) to go HIGH, signaling that the SPI frame has been fully processed and the division algorithm has successfully converged on a stable ratio.

## External hardware

To utilize this signal processing engine, you will need:

* Tiny Tapeout Demo Board (or equivalent carrier board).
* An external optical sensor frontend (like MAX30102) with an SPI master/microcontroller to feed the raw Red/IR digital samples into the input pins.
* A logic analyzer or the Tiny Tapeout Commander app to stream inputs and observe the computed ratio flags.
