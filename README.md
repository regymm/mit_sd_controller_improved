## mit_sd_controller_improved
Improved version of http://web.mit.edu/6.111/volume2/www/f2018/tools/sd_controller.v

... and more. 

### Improvements

- SDHC card support: send CMD8 after CMD0 and read the full R7 response. Change ACMD41 parameter to show host SDHC support. 
- Send about 74 <400KHz pulses to initialize, instead of default which wait for a few seconds.
- Use a normal speed clock and a slow pulsed clock instead of clock dividers. Good for timing analysis. 
- Initialization/Read/Write tested OK on a SanDisk 16GB SDHC card -- works on my machine! 

### sd_controller.v usage

Interfaces are the same as the original except clocks. 

`clk` is the main, fast clock and `clk_pulse_slow` is the pulsed, slow SPI clock. 

My `clk` is 62.5MHz and `clk_pulse_slow` is generated by:

```verilog
// slow clock
reg [4:0]clkcounter = 0;
always @ (posedge clk) begin
    if (rst) clkcounter <= 5'b0;
    else clkcounter <= clkcounter + 1;
end
wire clk_pulse_slow = (clkcounter == 5'b0);
```

If you use a different main clock or slow  pulse frequency, you may want to change the boot counter:

```verilog
boot_counter <= 27'd005_000;
```

and this: 

```verilog
if (boot_counter[2]) sclk_sig <= ~sclk_sig;
```

to make sure more than 74 cycles are sent slower than 400KHz during initialization. 

References are at the top of `sd_controller.v`. 

---

 below are my own stuffs 

---

### sdcard.v wrapper

This wrapper(written by me) can provide an easy memory-like interface to the sdcard. Can be used as memory-mapped IO for home-brew FPGA CPUs. Has 512 bytes cache(will be synthesis into block RAM). 

Interface: 

```verilog
input [15:0]a,
input [31:0]d,
input we,
output reg [31:0]spo,
```

Usage: 

```
read/write 0x0000 to 0x01fc: 128*32 block cache
read/write 0x1000: get/set <address> for R/W, auto 512 aligned (may purge existing cache)
write 0x1004: do a read at <address> (may purge existing cache)
write 0x1008: do a write to <address> (write cache to sector)
read 0x2000: negative card detect
read 0x2004: write protected
read 0x2010: ready, used for polling
read 0x2014: cache dirty?
```

For example write 0xdeadbeef to the beginning of sector 16:

```
write 0x10 to 0x1000
write 1 to 0x1004
read 0x2010 until get 1
write 0xdeadbeef to 0x0000
write 1 to 0x1008
read 0x2010 until get 1
```

### Circuit connection

Theoretically this will work:

```
 3.3V          ----+++              wp
                   |||         +---- =-----
                   RRR         =8
 MISO(SD_DAT[0]) --|||---------=7
                   |||  GND ---=6
 SCLK(SD_CLK)    --||+---------=5
                   ||   VCC ---=4
                   ||   GND ---=3
 MOSI(SD_CMD)    --|+----------=2
 CS(SD_DAT[3])   --+-----------=1
                                 +=9
                                   +-------
```

My PMOD extension board is different from this but also works. 

### License

I don't know the license of the original file, but I have seen this file in some github repos with different licenses... 

So for what I wrote, WTFPL. 