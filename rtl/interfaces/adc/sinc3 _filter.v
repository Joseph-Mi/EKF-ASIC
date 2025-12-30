module sinc3_filter_adc (
    input wire mclk1,              // Modulator clock (e.g., 20 MHz)
    input wire reset,              // Active-high reset
    input wire mdata1,             // 1-bit input stream from modulator
    output reg [15:0] DATA,        // 16-bit filtered output
    output reg data_en,            // Data valid strobe
    input wire [15:0] dec_rate     // Decimation rate
);

    // ============================================================
    // SIGNAL DECLARATIONS
    // ============================================================
    
    // Input conversion: 0 → 0, 1 → 1 (for unsigned)
    // Alternative: 0 → -1, 1 → +1 (for signed, uncomment below)
    reg [36:0] ip_data1;
    
    // Integrator (accumulator) registers - 3 stages
    reg [36:0] acc1;
    reg [36:0] acc2;
    reg [36:0] acc3;
    
    // Differentiator registers - 3 stages
    reg [36:0] acc3_d2;    // Delayed acc3 for differentiation
    reg [36:0] diff1;
    reg [36:0] diff2;
    reg [36:0] diff3;
    reg [36:0] diff1_d;    // Delayed diff1
    reg [36:0] diff2_d;    // Delayed diff2
    
    // Decimation control
    reg [15:0] word_count;
    reg word_clk;
    reg enable;
    
    // ============================================================
    // STAGE 1: INPUT DATA CONVERSION
    // ============================================================
    // Convert 1-bit input to appropriate value
    // For unsigned: 0→0, 1→1
    // For signed (two's complement): uncomment the -1 line below
    
    always @(mdata1) begin
        if (mdata1 == 0)
            ip_data1 <= 37'd0;
            // ip_data1 <= -37'd1;  // Uncomment for signed operation
        else
            ip_data1 <= 37'd1;
    end
    
    // ============================================================
    // STAGE 2: INTEGRATOR (ACCUMULATOR) - IIR SECTION
    // ============================================================
    // This runs at the full modulator speed (e.g., 20 MHz)
    // Three cascaded accumulators sum the input over time
    //
    // Transfer function for each stage: H(z) = 1 / (1 - z^-1)
    // Combined: H(z) = 1 / (1 - z^-1)^3
    
    always @(negedge mclk1 or posedge reset) begin
        if (reset) begin
            acc1 <= 37'd0;
            acc2 <= 37'd0;
            acc3 <= 37'd0;
        end else begin
            acc1 <= acc1 + ip_data1;   // First integrator
            acc2 <= acc2 + acc1;        // Second integrator
            acc3 <= acc3 + acc2;        // Third integrator
        end
    end
    
    // ============================================================
    // STAGE 3: DECIMATION CONTROL
    // ============================================================
    // Generate a slower clock (word_clk) based on decimation rate
    // This reduces the output data rate
    //
    // Example: 20 MHz / 256 = 78.125 kHz output rate
    
    always @(posedge mclk1 or posedge reset) begin
        if (reset) begin
            word_count <= 16'd0;
        end else begin
            if (word_count == dec_rate - 1)
                word_count <= 16'd0;
            else
                word_count <= word_count + 16'd1;
        end
    end
    
    // Generate word_clk: high for first half, low for second half
    always @(posedge mclk1 or posedge reset) begin
        if (reset) begin
            word_clk <= 1'b0;
        end else begin
            if (word_count == dec_rate/2 - 1)
                word_clk <= 1'b1;
            else if (word_count == dec_rate - 1)
                word_clk <= 1'b0;
        end
    end
    
    // ============================================================
    // STAGE 4: DIFFERENTIATOR (COMB) - FIR SECTION
    // ============================================================
    // This runs at the decimated rate (slower)
    // Three cascaded differentiators calculate differences
    //
    // Transfer function for each stage: H(z) = (1 - z^-D)
    // Combined: H(z) = (1 - z^-D)^3, where D = decimation rate
    
    always @(posedge word_clk or posedge reset) begin
        if (reset) begin
            acc3_d2 <= 37'd0;
            diff1_d <= 37'd0;
            diff2_d <= 37'd0;
            diff1 <= 37'd0;
            diff2 <= 37'd0;
            diff3 <= 37'd0;
        end else begin
            // First differentiator
            diff1 <= acc3 - acc3_d2;
            
            // Second differentiator
            diff2 <= diff1 - diff1_d;
            
            // Third differentiator
            diff3 <= diff2 - diff2_d;
            
            // Update delay registers
            acc3_d2 <= acc3;
            diff1_d <= diff1;
            diff2_d <= diff2;
        end
    end
    
    // ============================================================
    // STAGE 5: OUTPUT SCALING AND CLIPPING
    // ============================================================
    // Extract the correct bits based on decimation rate
    // The output width grows with log2(dec_rate) * filter_order
    // We select 16 bits from the appropriate position
    
    always @(posedge word_clk) begin
        case (dec_rate)
            16'd32: begin
                // 15 bits output, shift to 16 bits
                DATA <= (diff3[15:0] == 16'h8000) ? 16'hFFFF : {diff3[14:0], 1'b0};
            end
            16'd64: begin
                // 18 bits output, take middle 16 bits
                DATA <= (diff3[18:2] == 17'h10000) ? 16'hFFFF : diff3[17:2];
            end
            16'd128: begin
                // 21 bits output, take middle 16 bits
                DATA <= (diff3[21:5] == 17'h10000) ? 16'hFFFF : diff3[20:5];
            end
            16'd256: begin
                // 24 bits output, take middle 16 bits
                DATA <= (diff3[24:8] == 17'h10000) ? 16'hFFFF : diff3[23:8];
            end
            16'd512: begin
                // 27 bits output, take middle 16 bits
                DATA <= (diff3[27:11] == 17'h10000) ? 16'hFFFF : diff3[26:11];
            end
            16'd1024: begin
                // 30 bits output, take middle 16 bits
                DATA <= (diff3[30:14] == 17'h10000) ? 16'hFFFF : diff3[29:14];
            end
            16'd2048: begin
                // 33 bits output, take middle 16 bits
                DATA <= (diff3[33:17] == 17'h10000) ? 16'hFFFF : diff3[32:17];
            end
            16'd4096: begin
                // 36 bits output, take middle 16 bits
                DATA <= (diff3[36:20] == 17'h10000) ? 16'hFFFF : diff3[35:20];
            end
            default: begin
                // Default to 256 decimation
                DATA <= (diff3[24:8] == 17'h10000) ? 16'hFFFF : diff3[23:8];
            end
        endcase
    end
    
    // ============================================================
    // STAGE 6: DATA VALID STROBE GENERATION
    // ============================================================
    // Generate a pulse when new data is available
    // This synchronizes to the main clock domain
    
    always @(posedge mclk1 or posedge reset) begin
        if (reset) begin
            data_en <= 1'b0;
            enable <= 1'b1;
        end else begin
            if ((word_count == dec_rate/2 - 1) && enable) begin
                data_en <= 1'b1;
                enable <= 1'b0;
            end else if ((word_count == dec_rate - 1) && ~enable) begin
                data_en <= 1'b0;
                enable <= 1'b1;
            end else begin
                data_en <= 1'b0;
            end
        end
    end

endmodule
