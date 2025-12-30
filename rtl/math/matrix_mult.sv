// Description:
//   Optimized matrix multiplier using multiply-accumulate (MAC) units.
//   Computes C = A * B where:
//     A is N×M
//     B is M×P
//     C is N×P
//
// Features:
//   - Pipelined MAC for efficiency
//   - Configurable matrix dimensions
//   - Memory interface for reading A and B matrices
//   - Can be used for various EKF operations
//
// Typical Usage in EKF:
//   - P = F * P * F^T  (covariance prediction)
//   - K = P * H^T      (Kalman gain computation)
//   - x = F * x        (state prediction)
//
// Performance:
//   Cycles = N * P * (M + 3)  approximately
//   For 4x4 matrices: ~112 cycles
//
// Author: Joseph Mi
// Date: Dec 30, 2025
// =============================================================================

module matrix_mult
    import fp_arith_pkg::*; 
#(
    parameter int N = 4,  // Rows of A
    parameter int M = 4,  // Cols of A, Rows of B
    parameter int P = 4   // Cols of B
)(
    input  logic clk,
    input  logic rst_n,
    
    // Control
    input  logic start,
    output logic done,
    output logic busy,
    
    // Matrix A input (N×M)
    input  logic signed [DATA_WIDTH-1:0] matrix_a [N-1:0][M-1:0],
    
    // Matrix B input (M×P)
    input  logic signed [DATA_WIDTH-1:0] matrix_b [M-1:0][P-1:0],
    
    // Matrix C output (N×P)
    output logic signed [DATA_WIDTH-1:0] matrix_c [N-1:0][P-1:0]
);

    // =========================================================================
    // STATE MACHINE
    // =========================================================================
    
    typedef enum logic [2:0] {
        IDLE,
        INIT_ROW,
        INIT_COL,
        MAC_COMPUTE,
        WRITE_RESULT,
        DONE_STATE
    } state_t;
    
    state_t state;
    
    // =========================================================================
    // COUNTERS AND INDICES
    // =========================================================================
    
    logic [$clog2(N)-1:0] row_idx;    // Current row of A
    logic [$clog2(P)-1:0] col_idx;    // Current column of B
    logic [$clog2(M)-1:0] k_idx;      // MAC iteration index
    
    // =========================================================================
    // MAC UNIT
    // =========================================================================
    
    // Accumulator for current element computation
    logic signed [2*DATA_WIDTH-1:0] mac_accumulator;
    logic signed [DATA_WIDTH-1:0] mac_result;
    
    // Current operands
    logic signed [DATA_WIDTH-1:0] a_element;
    logic signed [DATA_WIDTH-1:0] b_element;
    logic signed [2*DATA_WIDTH-1:0] product;
    
    // =========================================================================
    // DATAPATH
    // =========================================================================
    
    // Extract current element from A and B
    assign a_element = matrix_a[row_idx][k_idx];
    assign b_element = matrix_b[k_idx][col_idx];
    
    // Multiply and accumulate
    assign product = $signed(a_element) * $signed(b_element);
    
    // Extract result with proper fixed-point alignment
    assign mac_result = mac_accumulator[FRAC_BITS+DATA_WIDTH-1:FRAC_BITS];
    
    // =========================================================================
    // MAIN STATE MACHINE
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            busy <= 1'b0;
            row_idx <= '0;
            col_idx <= '0;
            k_idx <= '0;
            mac_accumulator <= '0;
            
            // Initialize output matrix to zero
            for (int i = 0; i < N; i++) begin
                for (int j = 0; j < P; j++) begin
                    matrix_c[i][j] <= FP_ZERO;
                end
            end
            
        end else begin
            case (state)
                
                // --------------------------------------------------------
                // IDLE: Wait for start signal
                // --------------------------------------------------------
                IDLE: begin
                    done <= 1'b0;
                    busy <= 1'b0;
                    
                    if (start) begin
                        busy <= 1'b1;
                        row_idx <= '0;
                        col_idx <= '0;
                        state <= INIT_ROW;
                    end
                end
                
                // --------------------------------------------------------
                // INIT_ROW: Start new row computation
                // --------------------------------------------------------
                INIT_ROW: begin
                    col_idx <= '0;
                    state <= INIT_COL;
                end
                
                // --------------------------------------------------------
                // INIT_COL: Start new column computation
                // --------------------------------------------------------
                INIT_COL: begin
                    k_idx <= '0;
                    mac_accumulator <= '0;
                    state <= MAC_COMPUTE;
                end
                
                // --------------------------------------------------------
                // MAC_COMPUTE: Perform multiply-accumulate
                // C[row][col] = Σ(A[row][k] * B[k][col])
                // --------------------------------------------------------
                MAC_COMPUTE: begin
                    if (k_idx < M) begin
                        // Accumulate: mac += a[row][k] * b[k][col]
                        mac_accumulator <= mac_accumulator + product;
                        k_idx <= k_idx + 1'b1;
                    end else begin
                        // MAC complete for this element
                        state <= WRITE_RESULT;
                    end
                end
                
                // --------------------------------------------------------
                // WRITE_RESULT: Store computed element to output matrix
                // --------------------------------------------------------
                WRITE_RESULT: begin
                    // Check for overflow and saturate if needed
                    if (mac_accumulator[2*DATA_WIDTH-1:FRAC_BITS+DATA_WIDTH-1] != 
                        {(DATA_WIDTH-FRAC_BITS){mac_accumulator[2*DATA_WIDTH-1]}}) begin
                        // Overflow - saturate
                        matrix_c[row_idx][col_idx] <= mac_accumulator[2*DATA_WIDTH-1] ? 
                                                       FP_MIN_NEG : FP_MAX_POS;
                    end else begin
                        // No overflow
                        matrix_c[row_idx][col_idx] <= mac_result;
                    end
                    
                    // Move to next element
                    if (col_idx < P - 1) begin
                        // More columns in this row
                        col_idx <= col_idx + 1'b1;
                        state <= INIT_COL;
                    end else if (row_idx < N - 1) begin
                        // More rows to process
                        row_idx <= row_idx + 1'b1;
                        state <= INIT_ROW;
                    end else begin
                        // All elements computed
                        state <= DONE_STATE;
                    end
                end
                
                // --------------------------------------------------------
                // DONE_STATE: Signal completion
                // --------------------------------------------------------
                DONE_STATE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= IDLE;
                end
                
            endcase
        end
    end
    
    // =========================================================================
    // PERFORMANCE COUNTER (optional, for debugging)
    // =========================================================================
    
    `ifdef DEBUG_MATRIX_MULT
    logic [15:0] cycle_count;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle_count <= '0;
        end else begin
            if (start) begin
                cycle_count <= '0;
            end else if (busy) begin
                cycle_count <= cycle_count + 1'b1;
            end
        end
    end
    `endif

endmodule


// =============================================================================
// Matrix-Vector Multiplication (Specialized, Faster)
// =============================================================================
// Optimized for A * x where x is a column vector
// Commonly used in: x_pred = F * x

module matrix_vector_mult
    import fp_arith_pkg::*;
#(
    parameter int N = 4,  // Rows of A
    parameter int M = 4   // Cols of A (also size of vector x)
)(
    input  logic clk,
    input  logic rst_n,
    
    // Control
    input  logic start,
    output logic done,
    
    // Matrix A input (N×M)
    input  logic signed [DATA_WIDTH-1:0] matrix_a [N-1:0][M-1:0],
    
    // Vector x input (M×1)
    input  logic signed [DATA_WIDTH-1:0] vector_x [M-1:0],
    
    // Vector result output (N×1)
    output logic signed [DATA_WIDTH-1:0] vector_result [N-1:0]
);

    typedef enum logic [1:0] {
        IDLE,
        COMPUTE_ROW,
        DONE_STATE
    } state_t;
    
    state_t state;
    
    logic [$clog2(N)-1:0] row_idx;
    logic [$clog2(M)-1:0] col_idx;
    logic signed [2*DATA_WIDTH-1:0] accumulator;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            row_idx <= '0;
            col_idx <= '0;
            accumulator <= '0;
            
            for (int i = 0; i < N; i++) begin
                vector_result[i] <= FP_ZERO;
            end
            
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        row_idx <= '0;
                        col_idx <= '0;
                        accumulator <= '0;
                        state <= COMPUTE_ROW;
                    end
                end
                
                COMPUTE_ROW: begin
                    if (col_idx < M) begin
                        // result[row] += A[row][col] * x[col]
                        accumulator <= accumulator + 
                                       ($signed(matrix_a[row_idx][col_idx]) * 
                                        $signed(vector_x[col_idx]));
                        col_idx <= col_idx + 1'b1;
                    end else begin
                        // Store result for this row
                        vector_result[row_idx] <= accumulator[FRAC_BITS+DATA_WIDTH-1:FRAC_BITS];
                        
                        if (row_idx < N - 1) begin
                            row_idx <= row_idx + 1'b1;
                            col_idx <= '0;
                            accumulator <= '0;
                        end else begin
                            state <= DONE_STATE;
                        end
                    end
                end
                
                DONE_STATE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule