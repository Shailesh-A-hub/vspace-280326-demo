import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer, FallingEdge, RisingEdge, ClockCycles

@cocotb.test()
async def test_spo2_engine(dut):
        # Start the clock
        clock = Clock(dut.clk, 100, units="ns") # 10MHz
    cocotb.start_soon(clock.start())

    # Reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # Helper function to send an 8-bit sample via SPI
    async def send_spi_sample(sample, channel):
                # channel: 0=Red, 1=IR
                dut.ui_in[2].value = channel
                for i in range(7, -1, -1):
                                dut.ui_in[0].value = (sample >> i) & 1
                                await ClockCycles(dut.clk, 1)
                                dut.ui_in[1].value = 1 # SPI Clk High
            await ClockCycles(dut.clk, 1)
            dut.ui_in[1].value = 0 # SPI Clk Low
            await ClockCycles(dut.clk, 1)

    # Simulate some samples
    red_samples = [100, 110, 100, 90] * 64
    ir_samples = [200, 220, 200, 180] * 64

    for r, i in zip(red_samples, ir_samples):
                await send_spi_sample(r, 0) # Send Red
        await ClockCycles(dut.clk, 2)
        await send_spi_sample(i, 1) # Send IR
        await ClockCycles(dut.clk, 2)

    # Wait for processing
    await ClockCycles(dut.clk, 1000)

    # Check outputs
    spo2_val = dut.uo_out.value & 0x7F
    valid_flag = (dut.uo_out.value >> 7) & 1

    dut._log.info(f"SpO2 Output: {spo2_val}%, Valid: {valid_flag}")
