// EKF Control FSM - orchestrates predict and update stages
module ekf_control_fsm #(
    parameter STATE_DIM = 4,
    parameter MEAS_DIM = 2,
    parameter DATA_WIDTH = 32
)(
    input  logic clk,
    input  logic rst_n,
    
    // Command interface from SPI
    input  logic [7:0] cmd,
    input  logic [7:0] payload_length,
    input  logic [DATA_WIDTH-1:0] payload_data,
    input  logic payload_valid,
    output logic payload_ready,
    
    // Response interface to SPI
    output logic [DATA_WIDTH-1:0] response_data,
    output logic response_valid,
    input  logic response_ready,
    
    // Predict stage control
    output logic predict_start,
    input  logic predict_done,
    
    // Update stage control
    output logic update_start,
    input  logic update_done,
    
    // Memory interface
    output logic mem_wr_en,
    output logic [$clog2(STATE_DIM*STATE_DIM + STATE_DIM + MEAS_DIM)-1:0] mem_addr,
    output logic [DATA_WIDTH-1:0] mem_wr_data,
    input  logic [DATA_WIDTH-1:0] mem_rd_data
);

    // Command definitions
    localparam CMD_LOAD_STATE    = 8'h01;
    localparam CMD_LOAD_COV      = 8'h02;
    localparam CMD_LOAD_MEAS     = 8'h03;
    localparam CMD_RUN_PREDICT   = 8'h10;
    localparam CMD_RUN_UPDATE    = 8'h11;
    localparam CMD_RUN_FULL      = 8'h12;
    localparam CMD_READ_STATE    = 8'h20;
    localparam CMD_READ_COV      = 8'h21;

    typedef enum logic [3:0] {
        IDLE,
        LOAD_DATA,
        RUN_PREDICT,
        RUN_UPDATE,
        READ_RESULT,
        SEND_RESPONSE
    } state_t;
    
    state_t state, next_state;
    
    logic [7:0] cmd_reg;
    logic [$clog2(STATE_DIM*STATE_DIM)-1:0] data_cnt;
    logic [DATA_WIDTH-1:0] response_buffer;
    
    // State machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cmd_reg <= '0;
            data_cnt <= '0;
            response_buffer <= '0;
        end else begin
            state <= next_state;
            
            if (state == IDLE && payload_valid) begin
                cmd_reg <= cmd;
                data_cnt <= '0;
            end else if (state == LOAD_DATA && payload_valid) begin
                data_cnt <= data_cnt + 1'b1;
            end else if (state == READ_RESULT) begin
                response_buffer <= mem_rd_data;
            end
        end
    end
    
    // Next state logic
    always_comb begin
        next_state = state;
        
        case (state)
            IDLE: begin
                if (payload_valid) begin
                    case (cmd)
                        CMD_LOAD_STATE, CMD_LOAD_COV, CMD_LOAD_MEAS:
                            next_state = LOAD_DATA;
                        CMD_RUN_PREDICT:
                            next_state = RUN_PREDICT;
                        CMD_RUN_UPDATE:
                            next_state = RUN_UPDATE;
                        CMD_RUN_FULL:
                            next_state = RUN_PREDICT;
                        CMD_READ_STATE, CMD_READ_COV:
                            next_state = READ_RESULT;
                        default:
                            next_state = IDLE;
                    endcase
                end
            end
            
            LOAD_DATA: begin
                if (data_cnt == payload_length - 1 && payload_valid)
                    next_state = SEND_RESPONSE;
            end
            
            RUN_PREDICT: begin
                if (predict_done) begin
                    if (cmd_reg == CMD_RUN_FULL)
                        next_state = RUN_UPDATE;
                    else
                        next_state = SEND_RESPONSE;
                end
            end
            
            RUN_UPDATE: begin
                if (update_done)
                    next_state = SEND_RESPONSE;
            end
            
            READ_RESULT: begin
                if (data_cnt == payload_length - 1)
                    next_state = SEND_RESPONSE;
                else
                    next_state = READ_RESULT;
            end
            
            SEND_RESPONSE: begin
                if (response_valid && response_ready)
                    next_state = IDLE;
            end
        endcase
    end
    
    // Control outputs
    assign predict_start = (state == RUN_PREDICT) && (next_state != state);
    assign update_start = (state == RUN_UPDATE) && (next_state != state);
    
    assign mem_wr_en = (state == LOAD_DATA) && payload_valid;
    assign mem_addr = data_cnt;  // Simplified addressing
    assign mem_wr_data = payload_data;
    
    assign payload_ready = (state == LOAD_DATA) || (state == IDLE);
    
    assign response_data = response_buffer;
    assign response_valid = (state == SEND_RESPONSE);

endmodule