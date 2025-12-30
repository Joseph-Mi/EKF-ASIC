// EKF Parameters Package
// Defines all constants, types, and parameters used across the EKF system
package ekf_params_pkg;

    // ============================================================
    // SYSTEM PARAMETERS
    // ============================================================
    
    // Clock frequency
    parameter int SYSTEM_CLK_FREQ = 20_000_000;  // 20 MHz
    
    // EKF State dimensions
    parameter int STATE_DIM = 4;   // [SOC, SOH, V_terminal, I_pack]
    parameter int MEAS_DIM = 2;    // [V_measured, I_measured]
    parameter int CONTROL_DIM = 1; // [I_load]
    
    // ============================================================
    // MEMORY ORGANIZATION
    // ============================================================
    
    // Memory map:
    // [0    .. 3]    : State vector x (4 elements)
    // [4    .. 19]   : Covariance matrix P (4x4 = 16 elements)
    // [20   .. 35]   : Process noise Q (4x4 = 16 elements)
    // [36   .. 39]   : Measurement noise R (2x2 = 4 elements)
    // [40   .. 41]   : Measurement vector z (2 elements)
    // [42   .. 57]   : Jacobian F (4x4 = 16 elements)
    // [58   .. 65]   : Jacobian H (2x4 = 8 elements)
    // [66   .. 81]   : Kalman gain K (4x2 = 8 elements)
    // [82   .. 127]  : Scratch space for matrix operations
    
    parameter int ADDR_STATE_BASE = 0;
    parameter int ADDR_COV_BASE = 4;
    parameter int ADDR_Q_BASE = 20;
    parameter int ADDR_R_BASE = 36;
    parameter int ADDR_MEAS_BASE = 40;
    parameter int ADDR_JAC_F_BASE = 42;
    parameter int ADDR_JAC_H_BASE = 58;
    parameter int ADDR_KALMAN_BASE = 66;
    parameter int ADDR_SCRATCH_BASE = 82;
    
    parameter int TOTAL_MEM_SIZE = 128;  // Total memory locations
    parameter int MEM_ADDR_WIDTH = $clog2(TOTAL_MEM_SIZE);
    
    // ============================================================
    // SPI PROTOCOL
    // ============================================================
    
    // Command codes
    typedef enum logic [7:0] {
        CMD_NOP           = 8'h00,
        // Data loading commands
        CMD_LOAD_STATE    = 8'h01,
        CMD_LOAD_COV      = 8'h02,
        CMD_LOAD_Q        = 8'h03,
        CMD_LOAD_R        = 8'h04,
        CMD_LOAD_MEAS     = 8'h05,
        // Computation commands
        CMD_RUN_PREDICT   = 8'h10,
        CMD_RUN_UPDATE    = 8'h11,
        CMD_RUN_FULL_STEP = 8'h12,  // Predict + Update
        // Read commands
        CMD_READ_STATE    = 8'h20,
        CMD_READ_COV      = 8'h21,
        CMD_READ_SOC      = 8'h22,
        CMD_READ_SOH      = 8'h23,
        CMD_READ_VOLTAGE  = 8'h24,
        CMD_READ_CURRENT  = 8'h25,
        // Configuration commands
        CMD_SET_DT        = 8'h30,
        CMD_CALIBRATE_ADC = 8'h31,
        CMD_SET_DAC_MODE  = 8'h32,
        // Diagnostics
        CMD_READ_STATUS   = 8'hF0,
        CMD_READ_FAULT    = 8'hF1,
        CMD_RESET         = 8'hFF
    } ekf_command_t;
    
    // Status flags
    typedef struct packed {
        logic busy;
        logic adc_fault;
        logic spi_timeout;
        logic voltage_mismatch;
        logic predict_done;
        logic update_done;
        logic [1:0] data_source;  // 00=SPI, 01=ADC, 10=Both, 11=Fault
        logic heartbeat;
    } status_t;
    
    // Fault codes
    typedef enum logic [7:0] {
        FAULT_NONE            = 8'h00,
        FAULT_SPI_TIMEOUT     = 8'h10,
        FAULT_VOLTAGE_MISMATCH = 8'h20,
        FAULT_ADC_FAILURE     = 8'h30,
        FAULT_BOTH_SOURCES    = 8'hFF
    } fault_code_t;
    
    // ============================================================
    // ADC CONFIGURATION
    // ============================================================
    
    parameter int ADC_CHANNELS = 4;
    parameter int ADC_DECIMATION = 256;  // 20MHz / 256 = 78.125 kHz per channel
    parameter int ADC_OUTPUT_WIDTH = 16;
    
    // ADC channel mapping
    typedef enum logic [1:0] {
        ADC_CH_VPACK   = 2'b00,  // Pack voltage
        ADC_CH_IPACK   = 2'b01,  // Pack current
        ADC_CH_TEMP    = 2'b10,  // Temperature
        ADC_CH_VREF    = 2'b11   // Reference voltage (2.5V)
    } adc_channel_t;
    
    // ADC voltage thresholds (in ADC counts)
    parameter logic [15:0] ADC_VPACK_MIN = 16'd1000;   // ~30V
    parameter logic [15:0] ADC_VPACK_MAX = 16'd60000;  // ~360V
    parameter logic [15:0] ADC_VREF_NOMINAL = 16'd32768; // 2.5V reference
    
    // ============================================================
    // DAC CONFIGURATION
    // ============================================================
    
    parameter int DAC_CHANNELS = 4;
    parameter int PWM_RESOLUTION = 10;  // 10-bit PWM
    parameter int PWM_FREQ = 20_000;    // 20 kHz PWM frequency
    
    // DAC channel mapping
    typedef enum logic [1:0] {
        DAC_CH_SOC       = 2'b00,  // State of Charge output
        DAC_CH_SOH       = 2'b01,  // State of Health output
        DAC_CH_FAULT     = 2'b10,  // Fault status
        DAC_CH_HEARTBEAT = 2'b11   // 1 Hz heartbeat
    } dac_channel_t;
    
    // ============================================================
    // BATTERY MODEL PARAMETERS
    // ============================================================
    
    // Time step (0.1 seconds in Q16.16)
    parameter logic [DATA_WIDTH-1:0] DT = 32'h0000_1999;  // 0.1s
    
    // Battery capacity (in Ah, scaled)
    parameter logic [DATA_WIDTH-1:0] BATTERY_CAPACITY = 32'h0064_0000;  // 100 Ah
    
    // Initial state estimates
    parameter logic [DATA_WIDTH-1:0] INIT_SOC = 32'h0050_0000;  // 80% (0.8)
    parameter logic [DATA_WIDTH-1:0] INIT_SOH = 32'h0064_0000;  // 100% (1.0)
    parameter logic [DATA_WIDTH-1:0] INIT_VOLTAGE = 32'h012C_0000;  // 300V
    parameter logic [DATA_WIDTH-1:0] INIT_CURRENT = 32'h0000_0000;  // 0A
    
    // Process noise (Q matrix diagonal elements)
    parameter logic [DATA_WIDTH-1:0] Q_SOC = 32'h0000_0001;  // Very small
    parameter logic [DATA_WIDTH-1:0] Q_SOH = 32'h0000_0001;  // Very small (SOH changes slowly)
    parameter logic [DATA_WIDTH-1:0] Q_VOLTAGE = 32'h0000_0100;  // Moderate
    parameter logic [DATA_WIDTH-1:0] Q_CURRENT = 32'h0000_0100;  // Moderate
    
    // Measurement noise (R matrix diagonal elements)
    parameter logic [DATA_WIDTH-1:0] R_VOLTAGE = 32'h0000_1000;  // Sensor noise
    parameter logic [DATA_WIDTH-1:0] R_CURRENT = 32'h0000_1000;  // Sensor noise
    
    // Initial covariance (P matrix diagonal elements)
    parameter logic [DATA_WIDTH-1:0] P_INIT_SOC = 32'h0001_0000;      // High uncertainty initially
    parameter logic [DATA_WIDTH-1:0] P_INIT_SOH = 32'h0001_0000;
    parameter logic [DATA_WIDTH-1:0] P_INIT_VOLTAGE = 32'h0000_1000;
    parameter logic [DATA_WIDTH-1:0] P_INIT_CURRENT = 32'h0000_1000;
    
    // ============================================================
    // TIMING PARAMETERS
    // ============================================================
    
    parameter int PREDICT_CYCLES = 1000;   // Estimated cycles for predict operation
    parameter int UPDATE_CYCLES = 2000;    // Estimated cycles for update operation
    parameter int MATRIX_MULT_CYCLES = 200; // Per matrix multiply
    
    // Timeout values
    parameter int SPI_TIMEOUT_CYCLES = 2_000_000;  // 100ms @ 20MHz
    parameter int WATCHDOG_TIMEOUT = 20_000_000;   // 1 second
    
    // ============================================================
    // UTILITY FUNCTIONS
    // ============================================================
    
    // Convert SOC (0.0-1.0 in Q16.16) to percentage (0-100)
    function automatic logic [7:0] fp_to_percent(logic [DATA_WIDTH-1:0] fp_value);
        logic [DATA_WIDTH-1:0] scaled;
        scaled = (fp_value * 32'd100) >> FRAC_BITS;
        return scaled[7:0];
    endfunction
    
    // Convert percentage (0-100) to fixed-point (0.0-1.0)
    function automatic logic [DATA_WIDTH-1:0] percent_to_fp(logic [7:0] percent);
        return (percent << FRAC_BITS) / 32'd100;
    endfunction
    
    // Absolute value for signed fixed-point
    function automatic logic [DATA_WIDTH-1:0] fp_abs(logic signed [DATA_WIDTH-1:0] value);
        return (value < 0) ? -value : value;
    endfunction
    
    // Saturate to 16-bit range
    function automatic logic [15:0] saturate_16bit(logic signed [DATA_WIDTH-1:0] value);
        if (value > $signed(32'h7FFF_0000))
            return 16'h7FFF;
        else if (value < $signed(32'h8000_0000))
            return 16'h8000;
        else
            return value[FRAC_BITS+15:FRAC_BITS];
    endfunction

endpackage : ekf_params_pkg