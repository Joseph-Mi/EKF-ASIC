# EKF-ASIC: Extended Kalman Filter ASIC for Battery Management

A complete, modular SystemVerilog implementation of an Extended Kalman Filter (EKF) ASIC designed for battery management systems. Features redundant data paths and comprehensive I/O interfaces.

## 🎯 Key Features

### Core EKF Algorithm
- **4-state EKF**: SOC (State of Charge), SOH (State of Health), Terminal Voltage, Pack Current
- **Fixed-point arithmetic**: Q16.16 format for optimal hardware implementation
- **Modular design**: Separate predict, update, and Jacobian computation modules
- **Optimized matrix operations**: Pipelined multiply-accumulate (MAC) units

### Communication Interfaces

#### Primary: SPI Digital Interface
- **Mode**: Slave mode, Mode 0 (CPOL=0, CPHA=0)
- **Data width**: 32-bit
- **Oversampling**: 4x for reliability
- **Protocol**: Command-based with optional CRC16
- **Use case**: Main communication with MCU

#### Backup: Sigma-Delta ADC Inputs
- **Channels**: 4 independent channels
- **Resolution**: 16-bit per channel
- **Sampling rate**: 78.125 kHz (20 MHz / 256 decimation)
- **Filter**: 3rd-order Sinc filter (existing `sinc3_filter.v`)
- **Use case**: Backup data source if SPI fails, voltage cross-check

#### Monitoring: PWM DAC Outputs
- **Channels**: 2
- **Resolution**: 10-bit (0.1% effective)
- **Frequency**: 20 kHz PWM (requires external RC filter)
- **Outputs**: SOC, Heartbeat
- **Use case**: Real-time monitoring, oscilloscope debugging, watchdog

### Redundancy & Fault Tolerance
- **Automatic failover**: Switches from SPI to ADC if communication fails
- **Voltage cross-checking**: Compares SPI and ADC readings
- **Fault detection**: Tracks SPI timeout, voltage mismatch, ADC failures
- **Transparent operation**: EKF core always receives valid data

## 📁 Project Structure

```
EKF-ASIC/
├── rtl/                          # RTL source files
│   ├── top/
│   │   └── ekf_top.sv           # Top-level integration
│   ├── core/
│   │   ├── ekf_predict.sv       # Prediction step
│   │   ├── ekf_update.sv        # Update step
│   │   ├── ekf_control_fsm.sv   # Control state machine
│   │   └── ekf_state_machine.sv
│   ├── math/
│   │   ├── matrix_multiply.sv   # Pipelined matrix multiplier
│   │   ├── matrix_add.sv
│   │   ├── matrix_subtract.sv
│   │   ├── matrix_transpose.sv
│   │   ├── matrix_inverse_cholesky.sv
│   │   ├── jacobian_f.sv        # State transition Jacobian
│   │   ├── jacobian_h.sv        # Measurement Jacobian
│   │   └── fixed_point_ops.sv
│   ├── memory/
│   │   ├── dual_port_ram.sv     # Dual-port block RAM
│   │   ├── state_memory.sv
│   │   └── memory_arbiter.sv
│   ├── interfaces/
│   │   ├── spi/
│   │   │   ├── spi_slave.sv     # SPI slave interface
│   │   │   ├── spi_protocol.sv  # Packet protocol handler
│   │   │   └── spi_crc16.sv     # CRC computation
│   │   ├── adc/
│   │   │   ├── sigma_delta_adc_wrapper.sv  # ADC top-level
│   │   │   ├── sinc3_filter.sv  # Your existing filter
│   │   │   ├── adc_mux.sv
│   │   │   └── data_source_arbiter.sv
│   │   └── dac/
│   │       ├── pwm_dac.sv       # Simple PWM generator
│   │       ├── dac_controller.sv # Multi-channel controller
│   │       └── dac_filter.sv
│   └── packages/
│       ├── ekf_params_pkg.sv    # Parameters and types
│       ├── fixed_point_pkg.sv   # Fixed-point math functions
│       └── interface_pkg.sv
├── tb/                          # Testbenches
│   ├── unit_tests/
│   ├── integration/
│   └── testbench_utils/
├── docs/                        # Documentation
│   ├── PROJECT_STRUCTURE.md
│   ├── ADC_DAC_REDUNDANCY.md
│   ├── INTERFACE_SPEC.md
│   └── architecture.md
├── sim/                         # Simulation scripts
├── syn/                         # Synthesis scripts
└── README.md                    # This file
```

## 🚀 Getting Started

### Prerequisites
- SystemVerilog-compatible simulator (ModelSim, Questa, VCS, or Verilator)
- Synthesis tool (Synopsys Design Compiler, Cadence Genus, or Yosys)
- Python 3.8+ (for testbench automation)

### Quick Start

1. **Clone the repository**
   ```bash
   git clone <repo-url>
   cd EKF-ASIC
   ```

2. **Run unit tests**
   ```bash
   cd tb/unit_tests
   ./run_tests.sh
   ```

3. **Simulate the full system**
   ```bash
   cd sim
   make sim_ekf_top
   ```

4. **Synthesize for ASIC**
   ```bash
   cd syn
   make synthesize TARGET=<your_process>
   ```

## 🔧 Hardware Integration

### SPI Interface Connection (Primary Path)

```
┌─────────────┐                 ┌──────────────┐
│    MCU      │                 │  EKF-ASIC    │
│             │                 │              │
│  SPI_SCLK   ├────────────────►│  spi_sclk    │
│  SPI_MOSI   ├────────────────►│  spi_mosi    │
│  SPI_MISO   │◄────────────────┤  spi_miso    │
│  SPI_CS     ├────────────────►│  spi_cs      │
│             │                 │              │
└─────────────┘                 └──────────────┘
```

**SPI Configuration:**
- Mode 0 (CPOL=0, CPHA=0)
- Max clock: 5 MHz (with 20 MHz system clock and 4x oversampling)
- Data: MSB first, 32-bit words

### ADC Interface Connection (Backup Path)

```
Battery Pack (300V)
    │
    ├─[100kΩ]─┬─[1kΩ]──── (3.0V max) ──┐
    │         │                         │
    │       [TVS]                       │
    │         │                         │
    │        GND                        │
    │                                   │
    │                              ┌────▼─────┐
    │                              │ Comparator│
    │                              │  (Σ-Δ)    │
    └──────────────────────────────┤ -  │  out ├──► adc_vpack_raw
                                   │ +  │      │
                           2.5V ───┤    │      │
                           ref     └──────────┘
```

**ADC Channels:**
- CH0: Pack voltage (0-360V → 0-3.3V via resistor divider)
- CH1: Pack current (shunt voltage × 330 gain)
- CH2: Temperature (thermistor → voltage)
- CH3: 2.5V reference (for calibration)

### DAC Output Connection (Monitoring Path)

```
┌──────────────┐     RC Filter           ┌───────────┐
│  EKF-ASIC    │                         │  Monitor  │
│              │                         │           │
│ dac_soc_out  ├──[1kΩ]──┬──[10µF]─────►│ SOC Input │
│              │          │              │           │
│              │         GND             │           │
│              │                         │           │
│ dac_heartbeat├─────────────────────────►│ Watchdog  │
│              │                         │           │
└──────────────┘                         └───────────┘
```

**DAC Outputs:**
- SOC: 0-3.3V = 0-100% State of Charge
- SOH: 0-3.3V = 0-100% State of Health
- Fault: Voltage proportional to fault code
- Heartbeat: 1 Hz square wave when running

## 📊 Performance Specifications

| Parameter | Value |
|-----------|-------|
| System Clock | 20 MHz |
| SPI Max Clock | 5 MHz |
| ADC Sample Rate | 78.125 kHz (per channel) |
| ADC Resolution | 16 bits |
| DAC Resolution | 10 bits (effective) |
| DAC Update Rate | 100 Hz (for SOC/SOH) |
| EKF Predict Cycles | ~1000 (50 µs @ 20 MHz) |
| EKF Update Cycles | ~2000 (100 µs @ 20 MHz) |
| Full EKF Step | ~150 µs |
| Power Consumption | TBD (depends on process) |

## 🧮 Algorithm Details

### State Vector
The EKF estimates 4 states:

```
x = [SOC, SOH, V_terminal, I_pack]ᵀ
```

- **SOC**: State of Charge (0.0 to 1.0)
- **SOH**: State of Health (0.0 to 1.0)
- **V_terminal**: Terminal voltage (V)
- **I_pack**: Pack current (A)

### Measurement Vector
Two measurements are available:

```
z = [V_measured, I_measured]ᵀ
```

### Process Model
Simplified battery dynamics:

```
SOC(k+1) = SOC(k) - (I(k) * dt) / Capacity
SOH(k+1) = SOH(k)  (slowly varying)
V(k+1) = f(SOC, I, temperature)
I(k+1) = I(k)  (with process noise)
```

### Fixed-Point Format
- **Q16.16**: 16 integer bits, 16 fractional bits
- Range: -32768.0 to +32767.9999847
- Resolution: 0.0000152587890625 (1/65536)

## 🛠️ Configuration

### Compile-Time Parameters

Edit `rtl/packages/ekf_params_pkg.sv`:

```systemverilog
parameter int STATE_DIM = 4;        // Number of states
parameter int MEAS_DIM = 2;         // Number of measurements
parameter int ADC_CHANNELS = 4;     // Number of ADC inputs
parameter int DAC_CHANNELS = 4;     // Number of DAC outputs
parameter int ADC_DECIMATION = 256; // ADC decimation rate
```

### Runtime Configuration (via SPI)

```python
# Example Python code for MCU
spi.send_command(CMD_SET_DT, dt=0.1)           # Set time step
spi.send_command(CMD_LOAD_Q, Q_matrix)         # Process noise
spi.send_command(CMD_LOAD_R, R_matrix)         # Measurement noise
spi.send_command(CMD_CALIBRATE_ADC, offsets)   # ADC calibration
```

## 📈 Usage Example

### Basic Operation

```python
from ekf_asic import EKF_ASIC

# Initialize
ekf = EKF_ASIC(spi_device="/dev/spidev0.0")

# Load initial state
ekf.load_state(soc=0.8, soh=1.0, voltage=300.0, current=0.0)

# Load covariance and noise matrices
ekf.load_covariance(P_init)
ekf.load_process_noise(Q)
ekf.load_measurement_noise(R)

# Main loop
while True:
    # Get measurements from BMS
    voltage, current = bms.read_measurements()
    
    # Send measurements to EKF
    ekf.load_measurements(voltage, current)
    
    # Run EKF step
    ekf.run_full_step()  # Predict + Update
    
    # Read results
    soc, soh = ekf.read_state()
    print(f"SOC: {soc*100:.1f}%, SOH: {soh*100:.1f}%")
    
    # Monitor via DAC outputs
    # - Oscilloscope on dac_soc_out shows real-time SOC
    # - Heartbeat output confirms EKF is running
    
    time.sleep(0.1)  # 10 Hz update rate
```

## 🔍 Debugging

### Viewing DAC Outputs

Connect oscilloscope to DAC outputs:
- **CH1**: `dac_soc_out` → Real-time SOC (0-3.3V)
- **CH2**: `dac_heartbeat_out` → 1 Hz if running

### Fault Codes

| Voltage | Code | Meaning |
|---------|------|---------|
| 0.0V | 0x00 | No fault |
| ~0.5V | 0x10 | SPI timeout |
| ~1.0V | 0x20 | Voltage mismatch (SPI vs ADC) |
| ~1.5V | 0x30 | ADC failure |
| 3.3V | 0xFF | Both SPI and ADC failed |

### SPI Commands

```c
// Command definitions
#define CMD_LOAD_STATE     0x01
#define CMD_LOAD_COV       0x02
#define CMD_LOAD_MEAS      0x05
#define CMD_RUN_PREDICT    0x10
#define CMD_RUN_UPDATE     0x11
#define CMD_RUN_FULL_STEP  0x12
#define CMD_READ_STATE     0x20
#define CMD_READ_SOC       0x22
#define CMD_READ_STATUS    0xF0
```

## 🧪 Testing

### Unit Tests
```bash
cd tb/unit_tests
./run_tb_matrix_multiply.sh   # Test matrix operations
./run_tb_predict.sh            # Test prediction step
./run_tb_adc_dac.sh            # Test ADC/DAC interfaces
```

### Integration Test
```bash
cd tb/integration
./run_tb_ekf_top.sh            # Full system test
```

### Hardware-in-the-Loop (HIL)
```bash
python3 scripts/hil_test.py --board=<your_board>
```

## 🎓 References

1. **Kalman Filtering**: 
   - Kalman, R. E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
   
2. **Battery State Estimation**:
   - Plett, G. L. (2004). "Extended Kalman filtering for battery management systems"
   
3. **Fixed-Point Arithmetic**:
   - Yates, R. (2009). "Fixed-Point Arithmetic: An Introduction"
   
4. **Sigma-Delta ADC**:
   - Schreier, R., & Temes, G. C. (2005). "Understanding Delta-Sigma Data Converters"

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## ✉️ Contact

For questions or support, please open an issue on GitHub.
