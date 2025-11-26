# Three-Phase Induction Motor Power Analysis & Dynamic Simulation

A comprehensive Python application with Tkinter GUI for analyzing three-phase induction motors with real-time dynamic simulation capabilities.

## Problem Statement

Analysis of a three-phase delta-connected motor with the following test results:

### Test Data:
- **No Load Test:**
  - W1 = 3 kW, W2 = -5 kW (two wattmeter method)
  - Line Current = 25 A

- **Full Load Test:**
  - W1 = 39 kW, W2 = 23.5 kW
  - Line Current = 92 A
  - Speed = 1460 rpm

### Motor Specifications:
- Number of Poles: 4
- Supply Frequency: 50 Hz
- Stator Resistance (hot): 0.13 Ω per phase
- Connection: Delta

## Features

### 1. **Input Parameters Tab**
- Interactive input fields for all motor parameters
- Real-time adjustment sliders for each parameter
- Quick reset to default values
- Data export functionality

### 2. **Static Analysis Tab**
- Comprehensive calculation of motor parameters:
  - Synchronous speed and slip
  - Input/output power
  - Power factor (no load and full load)
  - Efficiency
  - Losses breakdown (stator copper, iron, friction & windage)
  - Torque calculations
- Detailed formatted report with all results

### 3. **Dynamic Simulation Tab**
- Real-time ODE solvers:
  - **RK45 (Runge-Kutta 4-5)**: High accuracy adaptive solver
  - **Euler Method**: Simple explicit solver
- Adjustable simulation parameters:
  - Supply voltage
  - Load torque
  - Simulation time
- Control buttons: Start, Stop, Reset
- Progress bar for simulation status
- Dynamic state variables:
  - d-q axis stator currents
  - d-q axis rotor fluxes
  - Rotor speed
  - Electromagnetic torque

### 4. **Visualization Tab**
Six comprehensive plots:
- Speed-Torque characteristic curve
- Efficiency vs Load
- Power Factor vs Load
- Current vs Load
- Power Flow diagram
- Torque vs Slip

### 5. **Full Analysis Tab**
Multiple sub-tabs for detailed analysis:

#### Performance Characteristics
- Combined plot of output power, input power, and torque vs load

#### Efficiency Analysis
- Dual-axis plot showing efficiency and power factor across load range

#### Losses Breakdown
- Pie chart showing distribution of losses
- Bar chart with detailed loss values:
  - Stator copper loss
  - Rotor copper loss
  - Iron loss
  - Friction and windage

#### Circle Diagram
- Classical circle diagram representation
- Operating points marked (no load and full load)
- Power factor reference lines

## Installation

### Prerequisites
```bash
Python 3.7 or higher
```

### Install Dependencies
```bash
pip install -r requirements.txt
```

Required packages:
- numpy (numerical computations)
- matplotlib (plotting)
- scipy (ODE solvers)
- tkinter (GUI - usually comes with Python)

## Usage

### Run the Application
```bash
python motor_power_analysis.py
```

### Workflow

1. **Input Parameters**
   - Navigate to "Input Parameters" tab
   - Adjust values using text fields or sliders
   - Click "Calculate" to compute all parameters

2. **View Static Analysis**
   - Go to "Static Analysis" tab to see detailed calculations
   - All results are formatted in a comprehensive report

3. **Run Dynamic Simulation**
   - Navigate to "Dynamic Simulation" tab
   - Set supply voltage, load torque, and simulation time
   - Choose solver method (RK45 recommended for accuracy)
   - Click "Start Simulation"
   - View real-time plots of motor dynamics

4. **Explore Visualizations**
   - "Visualization" tab shows six key performance plots
   - All plots update automatically when parameters change

5. **Full Analysis**
   - "Full Analysis" tab provides in-depth analysis
   - Navigate through sub-tabs for specific analyses

6. **Export Data**
   - Click "Export Data" button to save results to text file
   - Includes static analysis and dynamic simulation data

## Technical Details

### Calculations Performed

1. **Synchronous Speed**
   ```
   Ns = (120 × f) / P
   ```

2. **Slip**
   ```
   s = (Ns - Nr) / Ns
   ```

3. **Power from Two Wattmeter Method**
   ```
   P_total = W1 + W2
   tan(φ) = √3 × (W1 - W2) / (W1 + W2)
   Power Factor = cos(φ)
   ```

4. **Losses**
   - Stator copper loss: 3 × I²phase × Rs
   - No load losses: Iron loss + Friction & windage
   - Total losses = Stator Cu loss + No load losses

5. **Efficiency**
   ```
   η = (Pout / Pin) × 100%
   ```

6. **Torque**
   ```
   T = Pout / ω
   ```

### Dynamic Model

The application uses a 5th-order state-space model:
- State variables: [i_sd, i_sq, ψ_rd, ψ_rq, ω_r]
- Implements d-q reference frame transformation
- Models electromagnetic and mechanical dynamics
- Solves differential equations using:
  - RK45: 4th order Runge-Kutta with 5th order error estimation
  - Euler: Simple first-order explicit method

### GUI Features

- **Auto-scaling**: Window and all widgets automatically adjust to window size
- **Responsive Layout**: Grid-based layout with proper weight configuration
- **Multiple Tabs**: Organized interface for different analysis types
- **Interactive Controls**: Sliders and buttons for easy parameter adjustment
- **Real-time Updates**: Plots update automatically
- **Error Handling**: Comprehensive error messages and validation

## Results for Given Problem

### Calculated Values:
- **Synchronous Speed**: 1500 rpm
- **Slip at Full Load**: 2.67%
- **No Load Input Power**: -2 kW (leading pf)
- **Full Load Input Power**: 62.5 kW
- **Full Load Power Factor**: 0.839 (lagging)
- **Stator Cu Loss**: 1.78 kW
- **Output Power**: ≈58.7 kW
- **Efficiency**: ≈93.9%
- **Full Load Torque**: ≈384 Nm

## Advanced Features

### Practical Electrical Engineering Applications

1. **Motor Selection**: Compare different motor ratings and characteristics
2. **Efficiency Optimization**: Identify optimal operating load range
3. **Startup Analysis**: Simulate motor behavior during startup transients
4. **Load Variation Studies**: Analyze performance under varying loads
5. **Energy Audit**: Calculate energy consumption and losses
6. **Teaching Tool**: Excellent for educational purposes in electrical engineering

### Dynamic Simulation Capabilities

- **Transient Response**: Observe motor behavior during startup
- **Load Changes**: Simulate sudden load variations
- **Voltage Dips**: Study effect of supply voltage variations
- **Control System Design**: Use as plant model for controller design
- **Parameter Sensitivity**: Analyze effect of parameter variations

## Tips

1. **For Accurate Results**: Use actual measured values from motor tests
2. **Dynamic Simulation**: Start with RK45 solver for better accuracy
3. **Performance**: Euler method is faster but less accurate
4. **Visualization**: Resize window to get better view of plots
5. **Data Export**: Regularly export data for documentation

## Troubleshooting

### Common Issues

1. **Import Errors**: Install required packages using pip
2. **Display Issues**: Ensure tkinter is properly installed
3. **Slow Simulation**: Reduce simulation time or use Euler method
4. **Plot Not Showing**: Click "Calculate" button first

## Future Enhancements

Potential additions:
- Real-time data acquisition from actual motors
- More sophisticated motor models (thermal, saturation)
- Optimization algorithms for parameter identification
- Comparison with standards (IEEE, IEC)
- Report generation in PDF format
- Database for storing multiple motor profiles

## Author

Created as a comprehensive educational tool for electrical engineering students and professionals.

## License

Free to use for educational and research purposes.

## Acknowledgments

Based on classical induction motor theory and modern numerical methods for dynamic system simulation.
