// Dual-port Block RAM for EKF state and matrices
module memory 
    import ekf_params_pkg::*;
#(
    parameter MEM_SIZE = TOTAL_MEM_SIZE,
    parameter ADDR_WIDTH = $clog2(MEM_SIZE)
)(
    input  logic clk,
    input  logic rst_n,
    
    // Port A: Read/Write from control FSM
    input  logic [ADDR_WIDTH-1:0] addr_a,
    input  logic [DATA_WIDTH-1:0] data_a,
    input  logic we_a,
    output logic [DATA_WIDTH-1:0] q_a,
    
    // Port B: Read-only from computational units
    input  logic [ADDR_WIDTH-1:0] addr_b,
    output logic [DATA_WIDTH-1:0] q_b
);

    // Dual-port RAM array
    logic [DATA_WIDTH-1:0] mem [MEM_SIZE-1:0];
    
    // Port A: Read/Write
    always_ff @(posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= data_a;
        end
        q_a <= mem[addr_a];
    end
    
    // Port B: Read-only
    always_ff @(posedge clk) begin
        q_b <= mem[addr_b];
    end
    
    // Initialize with safe values
    initial begin
        for (int i = 0; i < MEM_SIZE; i++) begin
            mem[i] = FP_ZERO;
        end
        // Initialize diagonal of covariance to 1.0 (high uncertainty)
        mem[ADDR_COV_BASE + 0*STATE_DIM + 0] = FP_ONE;
        mem[ADDR_COV_BASE + 1*STATE_DIM + 1] = FP_ONE;
        mem[ADDR_COV_BASE + 2*STATE_DIM + 2] = FP_ONE;
        mem[ADDR_COV_BASE + 3*STATE_DIM + 3] = FP_ONE;
    end

endmodule
