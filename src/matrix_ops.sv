// Pipelined matrix multiplier using MAC units
// Computes C = A * B for matrices of size NxM * MxP
module matrix_ops
    import ekf_params_pkg::*;
    import fixed_point_pkg::*;
#(
    parameter N = 4,  // Rows of A
    parameter M = 4,  // Cols of A, Rows of B  
    parameter P = 4   // Cols of B
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    output logic done,
    
    // Memory interface for reading A and B
    output logic [$clog2(TOTAL_MEM_SIZE)-1:0] mem_addr,
    input  logic [DATA_WIDTH-1:0] mem_data,
    
    // Base addresses in memory
    input  logic [$clog2(TOTAL_MEM_SIZE)-1:0] addr_a_base,
    input  logic [$clog2(TOTAL_MEM_SIZE)-1:0] addr_b_base,
    input  logic [$clog2(TOTAL_MEM_SIZE)-1:0] addr_c_base,
    
    // Result write interface
    output logic [DATA_WIDTH-1:0] result_data,
    output logic result_valid,
    output logic [$clog2(TOTAL_MEM_SIZE)-1:0] result_addr
);

    typedef enum logic [2:0] {
        IDLE,
        LOAD_A_ROW,
        LOAD_B_COL,
        COMPUTE_MAC,
        WRITE_RESULT
    } state_t;
    
    state_t state;
    
    logic [$clog2(N)-1:0] row_idx;
    logic [$clog2(P)-1:0] col_idx;
    logic [$clog2(M)-1:0] k_idx;
    
    logic [DATA_WIDTH-1:0] a_row [M-1:0];
    logic [DATA_WIDTH-1:0] b_col [M-1:0];
    logic signed [2*DATA_WIDTH-1:0] accumulator;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            row_idx <= '0;
            col_idx <= '0;
            k_idx <= '0;
            accumulator <= '0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        state <= LOAD_A_ROW;
                        row_idx <= '0;
                        col_idx <= '0;
                    end
                end
                
                LOAD_A_ROW: begin
                    // Load row of A
                    if (k_idx < M) begin
                        mem_addr <= addr_a_base + row_idx * M + k_idx;
                        a_row[k_idx] <= mem_data;
                        k_idx <= k_idx + 1;
                    end else begin
                        k_idx <= '0;
                        state <= LOAD_B_COL;
                    end
                end
                
                LOAD_B_COL: begin
                    // Load column of B
                    if (k_idx < M) begin
                        mem_addr <= addr_b_base + k_idx * P + col_idx;
                        b_col[k_idx] <= mem_data;
                        k_idx <= k_idx + 1;
                    end else begin
                        k_idx <= '0;
                        accumulator <= '0;
                        state <= COMPUTE_MAC;
                    end
                end
                
                COMPUTE_MAC: begin
                    // Multiply-accumulate
                    if (k_idx < M) begin
                        accumulator <= accumulator + 
                            $signed(a_row[k_idx]) * $signed(b_col[k_idx]);
                        k_idx <= k_idx + 1;
                    end else begin
                        state <= WRITE_RESULT;
                    end
                end
                
                WRITE_RESULT: begin
                    result_data <= accumulator[DATA_WIDTH+FRAC_BITS-1:FRAC_BITS];
                    result_addr <= addr_c_base + row_idx * P + col_idx;
                    result_valid <= 1'b1;
                    
                    // Move to next element
                    if (col_idx < P - 1) begin
                        col_idx <= col_idx + 1;
                        k_idx <= '0;
                        state <= LOAD_B_COL;
                    end else if (row_idx < N - 1) begin
                        col_idx <= '0;
                        row_idx <= row_idx + 1;
                        k_idx <= '0;
                        state <= LOAD_A_ROW;
                    end else begin
                        done <= 1'b1;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end

endmodule
