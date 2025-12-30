// ============================================================
// FIXED-POINT REPRESENTATION
// ============================================================

parameter int DATA_WIDTH = 32;
parameter int FRAC_BITS = 16;  // Q16.16 format
parameter int INT_BITS = DATA_WIDTH - FRAC_BITS;

// Fixed-point constants
parameter logic [DATA_WIDTH-1:0] FP_ZERO = 32'h0000_0000;
parameter logic [DATA_WIDTH-1:0] FP_ONE  = 32'h0001_0000;  // 1.0 in Q16.16
parameter logic [DATA_WIDTH-1:0] FP_HALF = 32'h0000_8000;  // 0.5

// Battery-specific scaling
parameter logic [DATA_WIDTH-1:0] VOLTAGE_SCALE = 32'h0064_0000;  // 100.0 (for 300V → 3V scaling)
parameter logic [DATA_WIDTH-1:0] CURRENT_SCALE = 32'h014A_0000;  // 330.0 (for current sense amp)

// ============================================================
// FIXED-POINT FUNCTIONS
// ============================================================

// Fixed-point addition with saturation
function automatic logic [DATA_WIDTH-1:0] fp_add(
    logic signed [DATA_WIDTH-1:0] a,
    logic signed [DATA_WIDTH-1:0] b
);
    logic signed [DATA_WIDTH:0] result;  // Extra bit for overflow
    result = $signed(a) + $signed(b);
    
    // Saturate on overflow
    if (result[DATA_WIDTH:DATA_WIDTH-1] == 2'b01)  // Positive overflow
        return {1'b0, {(DATA_WIDTH-1){1'b1}}};  // Max positive
    else if (result[DATA_WIDTH:DATA_WIDTH-1] == 2'b10)  // Negative overflow
        return {1'b1, {(DATA_WIDTH-1){1'b0}}};  // Max negative
    else
        return result[DATA_WIDTH-1:0];
endfunction

// Fixed-point subtraction with saturation
function automatic logic [DATA_WIDTH-1:0] fp_sub(
    logic signed [DATA_WIDTH-1:0] a,
    logic signed [DATA_WIDTH-1:0] b
);
    logic signed [DATA_WIDTH:0] result;
    result = $signed(a) - $signed(b);
    
    // Saturate on overflow
    if (result[DATA_WIDTH:DATA_WIDTH-1] == 2'b01)
        return {1'b0, {(DATA_WIDTH-1){1'b1}}};
    else if (result[DATA_WIDTH:DATA_WIDTH-1] == 2'b10)
        return {1'b1, {(DATA_WIDTH-1){1'b0}}};
    else
        return result[DATA_WIDTH-1:0];
endfunction

// Fixed-point multiplication
// Result = (a * b) >> FRAC_BITS
function automatic logic [DATA_WIDTH-1:0] fp_mult(
    logic signed [DATA_WIDTH-1:0] a,
    logic signed [DATA_WIDTH-1:0] b
);
    logic signed [2*DATA_WIDTH-1:0] product;
    logic signed [DATA_WIDTH-1:0] result;
    
    product = $signed(a) * $signed(b);
    
    // Shift right by FRAC_BITS to restore fixed-point format
    result = product[FRAC_BITS+DATA_WIDTH-1:FRAC_BITS];
    
    // Check for overflow (optional saturation)
    if (product[2*DATA_WIDTH-1:FRAC_BITS+DATA_WIDTH-1] != {(DATA_WIDTH-FRAC_BITS){product[2*DATA_WIDTH-1]}})
    begin
        // Overflow occurred - saturate
        if (product[2*DATA_WIDTH-1])  // Negative
            result = {1'b1, {(DATA_WIDTH-1){1'b0}}};
        else  // Positive
            result = {1'b0, {(DATA_WIDTH-1){1'b1}}};
    end
    
    return result;
endfunction

// Fixed-point division (expensive - use sparingly!)
// Result = (a << FRAC_BITS) / b
function automatic logic [DATA_WIDTH-1:0] fp_div(
    logic signed [DATA_WIDTH-1:0] a,
    logic signed [DATA_WIDTH-1:0] b
);
    logic signed [2*DATA_WIDTH-1:0] dividend;
    logic signed [DATA_WIDTH-1:0] result;
    
    if (b == 0) begin
        // Division by zero - return max/min
        result = (a >= 0) ? {1'b0, {(DATA_WIDTH-1){1'b1}}} : {1'b1, {(DATA_WIDTH-1){1'b0}}};
    end else begin
        // Shift dividend left before division
        dividend = $signed(a) <<< FRAC_BITS;
        result = dividend / $signed(b);
    end
    
    return result;
endfunction