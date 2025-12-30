// =============================================================================
// Matrix Addition Module
// =============================================================================
//
// Description:
//   Element-wise matrix addition: C = A + B
//   All matrices must be same dimensions (ROWS × COLS)
//
// Operation:
//   C[i][j] = A[i][j] + B[i][j]  for all i, j
//
// Typical Usage in EKF:
//   - P_pred = F*P*F^T + Q  (final step of prediction)
//   - Innovation covariance: S = H*P*H^T + R
//
// Performance:
//   Cycles = ROWS × COLS  (one cycle per element)
//   For 4×4 matrix: 16 cycles
//
// Author: Joseph Mi
// Date: Dec 28, 2025
// =============================================================================

module matrix_add
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
    
    // Input matrices A and B (ROWS × COLS)
    input  logic signed [DATA_WIDTH-1:0] matrix_a [ROWS-1:0][COLS-1:0],
    input  logic signed [DATA_WIDTH-1:0] matrix_b [ROWS-1:0][COLS-1:0],
    
    // Output matrix C = A + B
    output logic signed [DATA_WIDTH-1:0] matrix_c [ROWS-1:0][COLS-1:0]
);

    // =========================================================================
    // STATE MACHINE
    // =========================================================================
    
    typedef enum logic [1:0] {
        IDLE,
        COMPUTE,
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
            for (int i = 0; i < ROWS; i++) begin
                for (int j = 0; j < COLS; j++) begin
                    matrix_c[i][j] <= FP_ZERO;
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
                        state <= COMPUTE;
                    end
                end
                
                // -------------------------------------------------------------
                // COMPUTE: Add element-by-element
                // -------------------------------------------------------------
                COMPUTE: begin
                    // Perform addition with saturation
                    matrix_c[row_idx][col_idx] <= fp_add(
                        matrix_a[row_idx][col_idx],
                        matrix_b[row_idx][col_idx]
                    );
                    
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