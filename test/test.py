import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer, FallingEdge, RisingEdge, ClockCycles

@cocotb.test()
async def test_spo2_engine(dut):
    # Start the clock (10 MHz)
    clock = Clock(dut.clk, 100, unit="ns")
    cocotb.start_soon(clock.start())

    # Initialize all inputs to known values BEFORE reset
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.ena.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # Helper: send an 8-bit sample via SPI using WHOLE-BUS writes
    # ui_in[0] = serial data, ui_in[1] = SPI clock, ui_in[2] = channel select
    async def send_spi_sample(sample, channel):
        ch_bit = (channel & 1) << 2  # bit 2
        for i in range(7, -1, -1):
            data_bit = (sample >> i) & 1
            # Set data with SPI clk LOW
            dut.ui_in.value = data_bit | ch_bit
            await ClockCycles(dut.clk, 1)
            # SPI clk HIGH (posedge triggers deserializer)
            dut.ui_in.value = data_bit | 0x02 | ch_bit
            await ClockCycles(dut.clk, 1)
            # SPI clk LOW
            dut.ui_in.value = data_bit | ch_bit
            await ClockCycles(dut.clk, 1)

    # Simulate samples: 256 red/IR pairs
    red_samples = [100, 110, 100, 90] * 64
    ir_samples = [200, 220, 200, 180] * 64

    for r, ir in zip(red_samples, ir_samples):
        await send_spi_sample(r, 0)   # Red channel
        await ClockCycles(dut.clk, 4)
        await send_spi_sample(ir, 1)  # IR channel
        await ClockCycles(dut.clk, 4)

    # Wait for vld (bit 7 of uo_out) to go high
    # The sequential multiplier + divider takes ~30 clocks, allow plenty of margin
    for _ in range(10000):
        val = dut.uo_out.value
        if val.is_resolvable and ((int(val) >> 7) & 1):
            break
        await RisingEdge(dut.clk)

    # Read outputs
    raw = int(dut.uo_out.value)
    spo2_val = raw & 0x7F
    valid_flag = (raw >> 7) & 1

    dut._log.info(f"SpO2 Output: {spo2_val}%, Valid: {valid_flag}")
    assert valid_flag == 1, "Calculation did not produce a valid output in time!"
