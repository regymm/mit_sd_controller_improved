`timescale 1ns / 1ps
// pComputer memory mapped SPI mode SD card driver
//
//  3.3V          ----+++              wp
//                    |||         +---- =-----
//                    RRR         =8
//  MISO(SD_DAT[0]) --|||---------=7
//                    |||  GND ---=6
//  SCLK(SD_CLK)    --||+---------=5
//                    ||   VCC ---=4
//                    ||   GND ---=3
//  MOSI(SD_CMD)    --|+----------=2
//  CS(SD_DAT[3])   --+-----------=1
//                                  +=9
//                                    +-------
//
// *addresses have already x4
// read/write 0x0000 to 0x01fc: 128*32 block cache
// read/write 0x1000: get/set <address> for R/W, auto 512 aligned (may lost changes)
// write 0x1004: do a read at <address> (may lost changes)
// write 0x1008: do a write to <address>
// read 0x2000: negative card detect
// read 0x2004: write protected
// read 0x2010: ready, used for polling
// read 0x2014: dirty?

module sdcard
    (
        input clk,
        //input clk_slow,
        input rst,

        input sd_dat0,
        input sd_ncd,
        input sd_wp,
        output sd_dat1,
        output sd_dat2,
        output sd_dat3,
        output sd_cmd,
        output sd_sck,
        // SD_DAT[2] and SD_DAT[1] high, SD_RESET low
        
        // memory interface
        input [15:0]a,
        input [31:0]d,
        input we,
        output reg [31:0]spo,

        output reg irq = 0
    );

    // slow clock
    reg [4:0]clkcounter = 0;
    always @ (posedge clk) begin
        if (rst) clkcounter <= 5'b0;
        else clkcounter <= clkcounter + 1;
    end
    wire clk_pulse_slow = (clkcounter == 5'b0);

    assign sd_dat1 = 1;
    assign sd_dat2 = 1;
    //assign sd_reset = 0;

    reg [31:0]sd_address = 32'hffffffff; // init as an invalid address
    reg [31:0]block[127:0];
    reg dirty = 0;

    reg sd_rd = 0;
    wire [7:0]sd_dout;
    wire sd_readnext;
    reg sd_wr = 0;
    reg [7:0]sd_din = 0;
    wire sd_writenext;
    wire sd_ready;
    wire [4:0]sd_status;
    sd_controller sd_controller_inst
    (
        .clk(clk),
        .clk_pulse_slow(clk_pulse_slow),
        .reset(rst),

        .cs(sd_dat3),
        .mosi(sd_cmd),
        .miso(sd_dat0),
        .sclk(sd_sck),

        .address(sd_address),

        .rd(sd_rd),
        .dout(sd_dout),
        .byte_available(sd_readnext),
        .wr(sd_wr),
        .din(sd_din),
        .ready_for_next_byte(sd_writenext),

        .ready(sd_ready),

        .status(sd_status)
    );

    // manual slow clock posedge detection
    reg sd_ready_old = 0;
    reg sd_readnext_old = 0;
    reg sd_writenext_old = 0;
    always @ (posedge clk) begin
        sd_ready_old <= sd_ready;
        sd_readnext_old <= sd_readnext;
        sd_writenext_old <= sd_writenext;
    end
    wire sd_ready_posedge = !sd_ready_old & sd_ready;
    wire sd_readnext_posedge = !sd_readnext_old & sd_readnext;
    wire sd_writenext_posedge = !sd_writenext_old & sd_writenext;

    wire sd_ready_real = sd_ready & !sd_rd & !sd_wr;

    reg reading = 0;
    reg writing = 0;
    reg [9:0]counter = 0;
    wire [31:0]cache = block[counter[8:2]];
    reg [7:0]cache1;
    reg [7:0]cache2;
    reg [7:0]cache3;
    always @ (posedge clk) begin
        if (rst) begin
            sd_address <= 32'hffffffff;
            dirty <= 0;
            reading <= 0;
            writing <= 0;
            sd_rd <= 0;
            sd_wr <= 0;
        end
        else begin
            if (sd_ready_real) begin
                if (we) begin
                    case (a[15:0])
                        16'h1000: sd_address <= d;
                        16'h1004: sd_rd <= d[0];
                        16'h1008: sd_wr <= d[0];
                        default: ;
                    endcase
                end
                // send/receive data has finished
                reading <= 0;
                writing <= 0;
            end
            else begin
                // sdcard has received signal and not IDLE
                // so we can stop rd/wr signals
                // and prepare to send/receive data
                if (sd_rd & !sd_ready) begin
                    sd_rd <= 0;
                    reading <= 1; counter <= 0;
                end
                else if (sd_wr & !sd_ready) begin
                    sd_wr <= 0;
                    writing <= 1; counter <= 0;
                    dirty <= 0;
                end
            end

            if (reading & sd_readnext_posedge) begin
                case (counter[1:0])
                    2'b00: cache1 <= sd_dout;
                    2'b01: cache2 <= sd_dout;
                    2'b10: cache3 <= sd_dout;
                    2'b11: block[counter[8:2]] <= {cache1, cache2, cache3, sd_dout};
                endcase
                counter <= counter + 1;
            end
            else if (writing & sd_writenext_posedge) begin
                case (counter[1:0])
                    2'b00: sd_din <= block[counter[8:2]][31:24];
                    2'b01: sd_din <= block[counter[8:2]][23:16];
                    2'b10: sd_din <= block[counter[8:2]][15:8];
                    2'b11: sd_din <= block[counter[8:2]][7:0];
                endcase
                counter <= counter + 1;
            end
            else if (we) begin
                if (a[15:12] == 0) begin
                    block[a[8:2]] <= d;
                    dirty <= 1;
                end
            end
        end
    end

    // handle non-relevant control address reading
    always @ (*) begin
        spo = 0;
        if (a[15:12] == 0) spo = block[a[8:2]];
        else case (a[15:0])
            16'h1000: spo = sd_address;
            16'h2000: spo = sd_ncd;
            16'h2004: spo = sd_wp;
            16'h2010: spo = sd_ready_real;
            16'h2014: spo = dirty;
            default: ;
        endcase
    end

    // interrupt when ready
    //assign irq = sd_ready_posedge;

endmodule

