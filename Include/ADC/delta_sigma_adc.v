module adc_system_example (
    input wire clk_20mhz,          // High-speed clock
    input wire rst_n,              // Active-low reset
    
    // Sigma-delta modulator inputs (1-bit streams)
    input wire adc_ch0_data,       // Channel 0: Current sensor
    input wire adc_ch1_data,       // Channel 1: Voltage sensor 1
    input wire adc_ch2_data,       // Channel 2: Voltage sensor 2
    input wire adc_ch3_data,       // Channel 3: Temperature sensor
    
    // Filtered outputs
    output wire [15:0] ch0_value,  // 16-bit filtered output
    output wire [15:0] ch1_value,
    output wire [15:0] ch2_value,
    output wire [15:0] ch3_value,
    
    // Data valid signals
    output wire ch0_valid,
    output wire ch1_valid,
    output wire ch2_valid,
    output wire ch3_valid
);

    wire reset;
    assign reset = ~rst_n;
    
    parameter DECIMATION = 16'd256;  // 20MHz / 256 = 78.125 kHz output

    // Instantiate 4 independent ADC channels
    sinc3_filter_adc ch0_adc (
        .mclk1(clk_20mhz),
        .reset(reset),
        .mdata1(adc_ch0_data),
        .DATA(ch0_value),
        .data_en(ch0_valid),
        .dec_rate(DECIMATION)
    );
    
    sinc3_filter_adc ch1_adc (
        .mclk1(clk_20mhz),
        .reset(reset),
        .mdata1(adc_ch1_data),
        .DATA(ch1_value),
        .data_en(ch1_valid),
        .dec_rate(DECIMATION)
    );
    
    sinc3_filter_adc ch2_adc (
        .mclk1(clk_20mhz),
        .reset(reset),
        .mdata1(adc_ch2_data),
        .DATA(ch2_value),
        .data_en(ch2_valid),
        .dec_rate(DECIMATION)
    );
    
    sinc3_filter_adc ch3_adc (
        .mclk1(clk_20mhz),
        .reset(reset),
        .mdata1(adc_ch3_data),
        .DATA(ch3_value),
        .data_en(ch3_valid),
        .dec_rate(DECIMATION)
    );

endmodule
