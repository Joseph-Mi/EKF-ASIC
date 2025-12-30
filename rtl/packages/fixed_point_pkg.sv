// Fixed-Point Arithmetic Package
// Provides functions for Q16.16 fixed-point math operations
package fixed_point_pkg;

    import ekf_params_pkg::*;
    
    // ============================================================
    // ADDITION AND SUBTRACTION
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
    
    // ============================================================
    // MULTIPLICATION
    // ============================================================
    
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
    
    // ============================================================
    // DIVISION
    // ============================================================
    
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
    
    // ============================================================
    // SQUARE ROOT (Newton-Raphson iteration)
    // ============================================================
    
    // Fixed-point square root approximation
    // Uses 4 iterations of Newton-Raphson: x_n+1 = (x_n + a/x_n) / 2
    function automatic logic [DATA_WIDTH-1:0] fp_sqrt(
        logic [DATA_WIDTH-1:0] a
    );
        logic [DATA_WIDTH-1:0] x, x_new;
        logic [DATA_WIDTH-1:0] a_div_x;
        int i;
        
        if (a == 0)
            return 32'h0;
        
        // Initial guess: x = a / 2
        x = a >> 1;
        if (x == 0) x = FP_ONE;  // Avoid zero initial guess
        
        // Newton-Raphson iterations
        for (i = 0; i < 4; i++) begin
            a_div_x = fp_div(a, x);
            x_new = fp_add(x, a_div_x);
            x = x_new >> 1;  // Divide by 2
        end
        
        return x;
    endfunction
    
    // ============================================================
    // TRIGONOMETRIC FUNCTIONS (Using Taylor series)
    // ============================================================
    
    // Sine approximation using Taylor series
    // sin(x) ≈ x - x³/6 + x⁵/120
    // Assumes input x in radians (Q16.16)
    function automatic logic signed [DATA_WIDTH-1:0] fp_sin(
        logic signed [DATA_WIDTH-1:0] x
    );
        logic signed [DATA_WIDTH-1:0] x_squared, x_cubed, x_fifth;
        logic signed [DATA_WIDTH-1:0] term1, term2, term3;
        
        // Compute powers of x
        x_squared = fp_mult(x, x);
        x_cubed = fp_mult(x_squared, x);
        x_fifth = fp_mult(x_cubed, x_squared);
        
        // Compute terms
        term1 = x;
        term2 = fp_div(x_cubed, 32'h0006_0000);  // x³/6
        term3 = fp_div(x_fifth, 32'h0078_0000);  // x⁵/120
        
        // sin(x) ≈ x - x³/6 + x⁵/120
        return fp_add(fp_sub(term1, term2), term3);
    endfunction
    
    // Cosine: cos(x) = sin(x + π/2)
    function automatic logic signed [DATA_WIDTH-1:0] fp_cos(
        logic signed [DATA_WIDTH-1:0] x
    );
        logic signed [DATA_WIDTH-1:0] pi_over_2;
        pi_over_2 = 32'h0001_921F;  // π/2 in Q16.16 (≈1.5708)
        return fp_sin(fp_add(x, pi_over_2));
    endfunction
    
    // ============================================================
    // EXPONENTIAL AND LOGARITHM (Simplified)
    // ============================================================
    
    // Natural exponential e^x using Taylor series
    // e^x ≈ 1 + x + x²/2 + x³/6 + x⁴/24
    function automatic logic [DATA_WIDTH-1:0] fp_exp(
        logic signed [DATA_WIDTH-1:0] x
    );
        logic signed [DATA_WIDTH-1:0] x_squared, x_cubed, x_fourth;
        logic signed [DATA_WIDTH-1:0] term1, term2, term3, term4, term5;
        logic signed [DATA_WIDTH-1:0] result;
        
        x_squared = fp_mult(x, x);
        x_cubed = fp_mult(x_squared, x);
        x_fourth = fp_mult(x_cubed, x);
        
        term1 = FP_ONE;
        term2 = x;
        term3 = fp_div(x_squared, 32'h0002_0000);  // x²/2
        term4 = fp_div(x_cubed, 32'h0006_0000);    // x³/6
        term5 = fp_div(x_fourth, 32'h0018_0000);   // x⁴/24
        
        result = fp_add(term1, term2);
        result = fp_add(result, term3);
        result = fp_add(result, term4);
        result = fp_add(result, term5);
        
        return result;
    endfunction
    
    // ============================================================
    // COMPARISON AND UTILITY
    // ============================================================
    
    // Absolute value
    function automatic logic [DATA_WIDTH-1:0] fp_abs(
        logic signed [DATA_WIDTH-1:0] x
    );
        return (x < 0) ? -x : x;
    endfunction
    
    // Maximum of two values
    function automatic logic [DATA_WIDTH-1:0] fp_max(
        logic signed [DATA_WIDTH-1:0] a,
        logic signed [DATA_WIDTH-1:0] b
    );
        return (a > b) ? a : b;
    endfunction
    
    // Minimum of two values
    function automatic logic [DATA_WIDTH-1:0] fp_min(
        logic signed [DATA_WIDTH-1:0] a,
        logic signed [DATA_WIDTH-1:0] b
    );
        return (a < b) ? a : b;
    endfunction
    
    // Clamp value between min and max
    function automatic logic [DATA_WIDTH-1:0] fp_clamp(
        logic signed [DATA_WIDTH-1:0] value,
        logic signed [DATA_WIDTH-1:0] min_val,
        logic signed [DATA_WIDTH-1:0] max_val
    );
        if (value < min_val)
            return min_val;
        else if (value > max_val)
            return max_val;
        else
            return value;
    endfunction
    
    // ============================================================
    // CONVERSION FUNCTIONS
    // ============================================================
    
    // Convert integer to fixed-point
    function automatic logic [DATA_WIDTH-1:0] int_to_fp(int value);
        return value << FRAC_BITS;
    endfunction
    
    // Convert fixed-point to integer (truncate)
    function automatic int fp_to_int(logic signed [DATA_WIDTH-1:0] value);
        return value >>> FRAC_BITS;
    endfunction
    
    // Convert floating-point bit pattern to fixed-point (for testbench)
    // NOTE: This is a simplified conversion, not full IEEE 754
    function automatic logic [DATA_WIDTH-1:0] float_to_fp(real f);
        return int'(f * (2.0 ** FRAC_BITS));
    endfunction
    
    // Convert fixed-point to floating-point (for testbench)
    function automatic real fp_to_float(logic signed [DATA_WIDTH-1:0] value);
        return real'($signed(value)) / (2.0 ** FRAC_BITS);
    endfunction
    
    // ============================================================
    // MATRIX UTILITIES
    // ============================================================
    
    // Get element from flattened matrix in row-major order
    function automatic int matrix_index(int row, int col, int num_cols);
        return row * num_cols + col;
    endfunction
    
    // Transpose index mapping
    function automatic int transpose_index(int idx, int rows, int cols);
        int row, col;
        row = idx / cols;
        col = idx % cols;
        return col * rows + row;
    endfunction

endpackage : fixed_point_pkg