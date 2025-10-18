// Top-level EKF ASIC module
module ekf_top #(
    parameter STATE_DIM = 4,
    parameter MEAS_DIM = 2,
    parameter DATA_WIDTH = 32
)(
    input  logic clk,
    input  logic rst_n,
    
    // SPI interface
    input  logic spi_sclk,
    input  logic spi_mosi,
    input  logic spi_cs_n,
    output logic spi_miso,
    
    // Optional status outputs
    output logic busy,
    output logic error
);

    // Internal signals
    logic [DATA_WIDTH-1:0] spi_rx_data, spi_tx_data;
    logic spi_rx_valid, spi_tx_ready;
    
    logic [7:0] cmd, payload_length;
    logic [DATA_WIDTH-1:0] payload_data, response_data;
    logic payload_valid, payload_ready;
    logic response_valid, response_ready;
    
    logic predict_start, predict_done;
    logic update_start, update_done;
    
    logic mem_wr_en;
    logic [$clog2(STATE_DIM*STATE_DIM + STATE_DIM + MEAS_DIM)-1:0] mem_addr;
    logic [DATA_WIDTH-1:0] mem_wr_data, mem_rd_data;
    
    // SPI Slave instance
    spi_slave #(
        .DATA_WIDTH(DATA_WIDTH),
        .OVERSAMPLE(4)
    ) u_spi_slave (
        .clk(clk),
        .rst_n(rst_n),
        .spi_sclk(spi_sclk),
        .spi_mosi(spi_mosi),
        .spi_cs_n(spi_cs_n),
        .spi_miso(spi_miso),
        .rx_data(spi_rx_data),
        .rx_valid(spi_rx_valid),
        .tx_data(spi_tx_data),
        .tx_ready(spi_tx_ready)
    );
    
    // SPI Protocol Handler
    spi_protocol #(
        .DATA_WIDTH(DATA_WIDTH),
        .MAX_PAYLOAD(16)
    ) u_spi_protocol (
        .clk(clk),
        .rst_n(rst_n),
        .spi_rx_data(spi_rx_data),
        .spi_rx_valid(spi_rx_valid),
        .spi_tx_data(spi_tx_data),
        .spi_tx_ready(spi_tx_ready),
        .cmd(cmd),
        .payload_length(payload_length),
        .payload_data(payload_data),
        .payload_valid(payload_valid),
        .payload_ready(payload_ready),
        .response_data(response_data),
        .response_valid(response_valid),
        .response_ready(response_ready)
    );
    
    // EKF Control FSM
    ekf_control_fsm #(
        .STATE_DIM(STATE_DIM),
        .MEAS_DIM(MEAS_DIM),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_ekf_fsm (
        .clk(clk),
        .rst_n(rst_n),
        .cmd(cmd),
        .payload_length(payload_length),
        .payload_data(payload_data),
        .payload_valid(payload_valid),
        .payload_ready(payload_ready),
        .response_data(response_data),
        .response_valid(response_valid),
        .response_ready(response_ready),
        .predict_start(predict_start),
        .predict_done(predict_done),
        .update_start(update_start),
        .update_done(update_done),
        .mem_wr_en(mem_wr_en),
        .mem_addr(mem_addr),
        .mem_wr_data(mem_wr_data),
        .mem_rd_data(mem_rd_data)
    );
    
    // Memory module (to be implemented)
    // Stores state vector, covariance matrix, measurements
    
    // Predict module (to be implemented in predict.sv)
    // Update module (to be implemented in update.sv)
    // Matrix operations (to be implemented in matrix_ops.sv)
    
    assign busy = (predict_start || update_start);
    assign error = 1'b0;  // Placeholder

endmodule