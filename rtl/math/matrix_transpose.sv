// =============================================================================
// Matrix Transpose Module
// =============================================================================
//
// Description:
//   Matrix transpose: C = A^T
//   Swaps rows and columns
//
// Operation:
//   C[i][j] = A[j][i]  for all i, j
//
// Typical Usage in EKF:
//   - F^T for covariance prediction: P = F*P*F^T
//   - H^T for Kalman gain: K = P*H^T*S^-1
//   - Symmetry verification: P should equal P^T
//
// Performance:
//   Cycles = ROWS × COLS  (one cycle per element)
//   For 4×4 matrix: 16 cycles
//
// Note:
//   Input dimensions: ROWS × COLS
//   Output dimensions: COLS × ROWS
//
// Author: Joseph Mi
// Date: Dec 2025
// =============================================================================

module matrix_transpose
    import fp_arith_pkg::*;
#(
    parameter int ROWS = 4,
    parameter int COLS = 4
)(
    input  logic clk,
    input  logic rst_n,
    
    // Control
    input  logic start,
    output logic done,
    output logic busy,
    
    // Input matrix A (ROWS × COLS)
    input  logic signed [DATA_WIDTH-1:0] matrix_in [ROWS-1:0][COLS-1:0],
    
    // Output matrix C = A^T (COLS × ROWS)
    output logic signed [DATA_WIDTH-1:0] matrix_out [COLS-1:0][ROWS-1:0]
);

    // =========================================================================
    // STATE MACHINE
    // =========================================================================
    
    typedef enum logic [1:0] {
        IDLE,
        TRANSPOSE,
        DONE_STATE
    } state_t;
    
    state_t state;
    
    // =========================================================================
    // COUNTERS
    // =========================================================================
    
    logic [$clog2(ROWS)-1:0] row_idx;
    logic [$clog2(COLS)-1:0] col_idx;
    
    // =========================================================================
    // MAIN FSM
    // =========================================================================
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            busy <= 1'b0;
            row_idx <= '0;
            col_idx <= '0;
            
            // Initialize output matrix
            for (int i = 0; i < COLS; i++) begin
                for (int j = 0; j < ROWS; j++) begin
                    matrix_out[i][j] <= FP_ZERO;
                end
            end
            
        end else begin
            case (state)
                
                // -------------------------------------------------------------
                // IDLE: Wait for start signal
                // -------------------------------------------------------------
                IDLE: begin
                    done <= 1'b0;
                    busy <= 1'b0;
                    
                    if (start) begin
                        busy <= 1'b1;
                        row_idx <= '0;
                        col_idx <= '0;
                        state <= TRANSPOSE;
                    end
                end
                
                // -------------------------------------------------------------
                // TRANSPOSE: Swap rows and columns
                // -------------------------------------------------------------
                TRANSPOSE: begin
                    // Transpose: out[col][row] = in[row][col]
                    matrix_out[col_idx][row_idx] <= matrix_in[row_idx][col_idx];
                    
                    // Move to next element
                    if (col_idx < COLS - 1) begin
                        col_idx <= col_idx + 1'b1;
                    end else if (row_idx < ROWS - 1) begin
                        col_idx <= '0;
                        row_idx <= row_idx + 1'b1;
                    end else begin
                        // All elements processed
                        state <= DONE_STATE;
                    end
                end
                
                // -------------------------------------------------------------
                // DONE_STATE: Signal completion
                // -------------------------------------------------------------
                DONE_STATE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= IDLE;
                end
                
            endcase
        end
    end

endmodule