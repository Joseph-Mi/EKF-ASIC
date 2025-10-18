// SPI packet protocol handler
// Packet format: [CMD][LENGTH][PAYLOAD...][CRC16_H][CRC16_L]
module spi_protocol #(
    parameter DATA_WIDTH = 32,
    parameter MAX_PAYLOAD = 16  // Maximum payload words
)(
    input  logic clk,
    input  logic rst_n,
    
    // SPI interface
    input  logic [DATA_WIDTH-1:0] spi_rx_data,
    input  logic spi_rx_valid,
    output logic [DATA_WIDTH-1:0] spi_tx_data,
    output logic spi_tx_ready,
    
    // Command interface to EKF
    output logic [7:0] cmd,
    output logic [7:0] payload_length,
    output logic [DATA_WIDTH-1:0] payload_data,
    output logic payload_valid,
    input  logic payload_ready,
    
    // Response interface from EKF
    input  logic [DATA_WIDTH-1:0] response_data,
    input  logic response_valid,
    output logic response_ready
);

    typedef enum logic [2:0] {
        IDLE,
        RX_CMD,
        RX_LENGTH,
        RX_PAYLOAD,
        RX_CRC,
        TX_RESPONSE
    } state_t;
    
    state_t state, next_state;
    
    // Registers
    logic [7:0] cmd_reg;
    logic [7:0] length_reg;
    logic [$clog2(MAX_PAYLOAD)-1:0] word_cnt;
    logic [15:0] crc_received;
    logic [15:0] crc_computed;
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cmd_reg <= '0;
            length_reg <= '0;
            word_cnt <= '0;
            crc_received <= '0;
        end else begin
            state <= next_state;
            
            case (state)
                RX_CMD: begin
                    if (spi_rx_valid) begin
                        cmd_reg <= spi_rx_data[7:0];
                    end
                end
                
                RX_LENGTH: begin
                    if (spi_rx_valid) begin
                        length_reg <= spi_rx_data[7:0];
                        word_cnt <= '0;
                    end
                end
                
                RX_PAYLOAD: begin
                    if (spi_rx_valid) begin
                        word_cnt <= word_cnt + 1'b1;
                    end
                end
                
                RX_CRC: begin
                    if (spi_rx_valid && word_cnt == 0) begin
                        crc_received[15:8] <= spi_rx_data[7:0];
                        word_cnt <= word_cnt + 1'b1;
                    end else if (spi_rx_valid && word_cnt == 1) begin
                        crc_received[7:0] <= spi_rx_data[7:0];
                    end
                end
            endcase
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (spi_rx_valid) next_state = RX_CMD;
            end
            
            RX_CMD: begin
                if (spi_rx_valid) next_state = RX_LENGTH;
            end
            
            RX_LENGTH: begin
                if (spi_rx_valid) begin
                    if (spi_rx_data[7:0] == 0)
                        next_state = RX_CRC;  // No payload
                    else
                        next_state = RX_PAYLOAD;
                end
            end
            
            RX_PAYLOAD: begin
                if (spi_rx_valid && (word_cnt == length_reg - 1))
                    next_state = RX_CRC;
            end
            
            RX_CRC: begin
                if (spi_rx_valid && word_cnt == 1)
                    next_state = TX_RESPONSE;
            end
            
            TX_RESPONSE: begin
                if (response_valid && response_ready)
                    next_state = IDLE;
            end
        endcase
    end
    
    // Output assignments
    assign cmd = cmd_reg;
    assign payload_length = length_reg;
    assign payload_data = spi_rx_data;
    assign payload_valid = (state == RX_PAYLOAD) && spi_rx_valid;
    
    assign spi_tx_data = response_data;
    assign spi_tx_ready = (state == TX_RESPONSE);
    assign response_ready = (state == TX_RESPONSE);

    // TODO: Add CRC16 computation module for data integrity
    assign crc_computed = 16'h0000;  // Placeholder

endmodule