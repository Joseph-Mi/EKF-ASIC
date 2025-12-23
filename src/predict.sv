// EKF Prediction Stage
module ekf_predict
    import ekf_params_pkg::*;
    import fixed_point_pkg::*;
(
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    output logic done,
    
    // Memory interface
    output logic [$clog2(TOTAL_MEM_SIZE)-1:0] mem_addr_rd,
    input  logic [DATA_WIDTH-1:0] mem_data_rd,
    output logic [$clog2(TOTAL_MEM_SIZE)-1:0] mem_addr_wr,
    output logic [DATA_WIDTH-1:0] mem_data_wr,
    output logic mem_we
);

    typedef enum logic [3:0] {
        IDLE,
        LOAD_STATE,
        COMPUTE_STATE_PRED,
        COMPUTE_JACOBIAN,
        MULT_A_P,      // A * P
        MULT_AP_AT,    // (A*P) * A^T
        ADD_Q,         // Result + Q
        DONE_STATE
    } state_t;
    
    state_t state;
    
    // State vector storage
    logic [DATA_WIDTH-1:0] x [STATE_DIM-1:0];
    logic [DATA_WIDTH-1:0] x_pred [STATE_DIM-1:0];
    
    // Jacobian A matrix (linearization)
    logic [DATA_WIDTH-1:0] A [STATE_DIM-1:0][STATE_DIM-1:0];
    
    // For simple constant-velocity model:
    // x_pred = [x + vx*dt, y + vy*dt, vx, vy]
    // A = [1  0  dt 0 ]
    //     [0  1  0  dt]
    //     [0  0  1  0 ]
    //     [0  0  0  1 ]
    
    parameter DT = 32'h0000_0666; // dt = 0.1s in Q16.16
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (start) begin
                        state <= LOAD_STATE;
                    end
                end
                
                LOAD_STATE: begin
                    // Load current state from memory
                    // ... (implement state loading)
                    state <= COMPUTE_STATE_PRED;
                end
                
                COMPUTE_STATE_PRED: begin
                    // x_pred = f(x)
                    // For constant velocity: x_pred[0] = x[0] + x[2]*dt
                    x_pred[0] <= fp_add(x[0], fp_mult(x[2], DT));
                    x_pred[1] <= fp_add(x[1], fp_mult(x[3], DT));
                    x_pred[2] <= x[2];  // vx unchanged
                    x_pred[3] <= x[3];  // vy unchanged
                    
                    state <= COMPUTE_JACOBIAN;
                end
                
                COMPUTE_JACOBIAN: begin
                    // For constant velocity, A is constant
                    A[0][0] <= FP_ONE;  A[0][1] <= FP_ZERO; A[0][2] <= DT;       A[0][3] <= FP_ZERO;
                    A[1][0] <= FP_ZERO; A[1][1] <= FP_ONE;  A[1][2] <= FP_ZERO;  A[1][3] <= DT;
                    A[2][0] <= FP_ZERO; A[2][1] <= FP_ZERO; A[2][2] <= FP_ONE;   A[2][3] <= FP_ZERO;
                    A[3][0] <= FP_ZERO; A[3][1] <= FP_ZERO; A[3][2] <= FP_ZERO;  A[3][3] <= FP_ONE;
                    
                    state <= MULT_A_P;
                end
                
                MULT_A_P: begin
                    // Trigger matrix multiplier: A * P
                    // ... (connect to matrix_mult module)
                    state <= MULT_AP_AT;
                end
                
                // ... (implement remaining states)
                
                DONE_STATE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule
