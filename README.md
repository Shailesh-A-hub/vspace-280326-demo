# V-SPACE Low-Power SpO2 Ratio Engine
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/tinytapeout/tt-setup/blob/main/tt-setup.ipynb)

Welcome to the V-SPACE Bootcamp! Click the badge above to launch the interactive chip design environment directly in your browser.

## Project Description: Low-Power SpO2 Ratio Engine
**Author:** A SHAILESH-24BEC0665

### Core Functionality:
This chip implements the digital signal processing core of a pulse oximeter -- a device that measures blood oxygen saturation (SpO2). It accepts alternating 8-bit Red (660nm) and IR (940nm) digitized photodiode samples via a serial input, extracts the AC (pulsatile) and DC (baseline) components of each channel, computes the perfusion ratio R = (AC_red / DC_red) / (AC_ir / DC_ir) using integer arithmetic, and outputs a 7-bit SpO2 percentage via a 16-entry calibration lookup table. No microcontroller or floating-point unit is required; the entire algorithm runs in dedicated hardware at uW-class power.

### Primary Logic Blocks:
*   **SPI Deserializer**: 8-bit shift register with channel-select flag; separates Red and IR samples on alternating clock cycles (~60 gates).
*   *   **DC Extractor x2**: 4-sample moving average using a shift register bank and adder tree with 2-bit right shift (divide by 4); one per channel (~180 gates).
    *   *   **AC Peak Holder x2**: 8-bit register updated when |sample - DC| exceeds current max; resets every 256 clock cycles (~100 gates).
        *   *   **Ratio Unit**: Computes R using cross-multiplication and bitshift-based division: R = (AC_red * DC_ir) >> (AC_ir * DC_red); implemented with shift-and-add multipliers (~200 gates).
            *   *   **16-Entry Calibration LUT**: Hardcoded case ROM mapping 4-bit R index to 7-bit SpO2 value (calibrated per Beer-Lambert empirical curve) (~112 gates).
             
                *   ### Staying Within the Gate Limit:
                *   Three deliberate constraints keep this design lean:
                *   1.  **4-sample averaging instead of 8**: Halves the shift register depth and adder complexity without significantly affecting DC accuracy at typical 100Hz PPG sampling rates.
                    2.  2.  **Bitshift-only arithmetic**: All multiplication and division uses power-of-2 approximations (shift + add), eliminating hardware multiplier blocks which typically cost 200-400 gates alone.
                        3.  3.  **4-bit LUT addressing**: Reduces the calibration ROM to 16 entries (7-bit wide), sufficient for clinical SpO2 accuracy of +/- 2% per ISO 80601-2-61.
                          
                            4.  ### Gate Budget Breakdown:
                            5.  *   **SPI Deserializer**: 60 gates
                                *   *   **DC Extractor**: 180 gates (2 channels)
                                    *   *   **AC Peak Holder**: 100 gates (2 channels)
                                        *   *   **Ratio Computation Unit**: 200 gates
                                            *   *   **16-Entry Calibration LUT**: 112 gates
                                                *   *   **Control FSM**: 80 gates (4-state Moore machine)
                                                    *   *   **Total Used**: 730 gates
                                                     
                                                        *   ---
                                                     
                                                        *   ## What is Tiny Tapeout?
                                                        *   Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.
                                                     
                                                        *   To learn more and get started, visit https://tinytapeout.com.
                                                     
                                                        *   ## Set up your Verilog project
                                                        *   1. Add your Verilog files to the src folder.
                                                            2. 2. Edit the info.yaml and update information about your project, paying special attention to the source_files and top_module properties.
                                                               3. 3. Edit docs/info.md and add a description of your project.
                                                                  4. 4. Adapt the testbench to your design. See test/README.md for more information.
                                                                     5. 5. The GitHub action will automatically build the ASIC files using LibreLane.
                                                                       
                                                                        6. ## Resources
                                                                        7. * [FAQ](https://tinytapeout.com/faq/)
                                                                           * * [Digital design lessons](https://tinytapeout.com/digital_design/)
                                                                             * * [Learn how semiconductors work](https://tinytapeout.com/hsw/)
                                                                               * * [Join the community](https://tinytapeout.com/discord)
                                                                                
                                                                                 * ## What next?
                                                                                 * * Submit your design to the next shuttle.
                                                                                   * * Edit this README and explain your design, how it works, and how to test it.
                                                                                     * * Share your project on your social network of choice:
                                                                                       *   * LinkedIn #tinytapeout @TinyTapeout
                                                                                           *   * Mastodon #tinytapeout @matthewvenn
                                                                                               *   * X (formerly Twitter) #tinytapeout @tinytapeout
                                                                                                   *   * Bluesky @tinytapeout.com
                                                                                                       * 
