// Simplified SPI slave with oversampling
module spi_slave #(
    parameter DATA_WIDTH = 32,
    parameter OVERSAMPLE = 4
)(
    input  logic clk,           // system clock
    input  logic rst_n,         // active low reset
    input  logic spi_sclk,      // SPI clock from MCU
    input  logic spi_mosi,      // SPI MOSI
    input  logic spi_cs_n,      // SPI CS (active low )
    output logic spi_miso,      // SPI MISO
    // Internal interface
    output logic [DATA_WIDTH-1:0] rx_data,
    output logic rx_valid,
    input  logic [DATA_WIDTH-1:0] tx_data,
    input  logic tx_ready
);
    // Synchronize SPI signals to system clock domain
    logic [2:0] sclk_sync, cs_sync, mosi_sync;
    
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            sclk_sync <= '0;
            cs_sync <= '1;
            mosi_sync <= '0;
        end else begin
            sclk_sync <= {sclk_sync[1:0], spi_sclk};
            cs_sync <= {cs_sync[1:0], spi_cs_n};
            mosi_sync <= {mosi_sync[1:0], spi_mosi};
        end
    end
    
    // Detect edges
    logic sclk_rising, sclk_falling, cs_falling;
    assign sclk_rising = (sclk_sync[2:1] == 2'b01);
    assign sclk_falling = (sclk_sync[2:1] == 2'b10);
    assign cs_falling = (cs_sync[2:1] == 2'b10);
    
// Bit counter
    logic [$clog2(DATA_WIDTH)-1:0] bit_cnt;
    logic transaction_active;
    
    // Shift registers
    logic [DATA_WIDTH-1:0] rx_shift_reg;
    logic [DATA_WIDTH-1:0] tx_shift_reg;
    
    // Transaction active when CS is low
    assign transaction_active = ~cs_sync[2];
    
    // RX shift register (sample on rising edge of SCLK)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_shift_reg <= '0;
            bit_cnt <= '0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;  // Default: pulse for one cycle
            
            if (cs_falling) begin
                // Start of transaction
                bit_cnt <= '0;
            end else if (transaction_active && sclk_rising) begin
                // Shift in data MSB first
                rx_shift_reg <= {rx_shift_reg[DATA_WIDTH-2:0], mosi_sync[2]};
                bit_cnt <= bit_cnt + 1'b1;
                
                // Check if complete word received
                if (bit_cnt == DATA_WIDTH - 1) begin
                    rx_valid <= 1'b1;
                    bit_cnt <= '0;
                end
            end else if (!transaction_active) begin
                bit_cnt <= '0;
            end
        end
    end
    
    assign rx_data = rx_shift_reg;
    
    // TX shift register (update on falling edge of SCLK)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift_reg <= '0;
            spi_miso <= 1'b0;
        end else begin
            if (cs_falling && tx_ready) begin
                // Load new data at start of transaction
                tx_shift_reg <= tx_data;
                spi_miso <= tx_data[DATA_WIDTH-1];
            end else if (transaction_active && sclk_falling) begin
                // Shift out data MSB first
                tx_shift_reg <= {tx_shift_reg[DATA_WIDTH-2:0], 1'b0};
                spi_miso <= tx_shift_reg[DATA_WIDTH-1];
            end else if (!transaction_active) begin
                spi_miso <= 1'b0;
            end
        end
    end

endmodule