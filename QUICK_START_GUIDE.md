# Quick Start Guide - Motor Power Analysis Application

## Installation & Running

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run the Application
```bash
python motor_power_analysis.py
```

## Using the Application

### Quick Analysis (5 minutes)

1. **Launch the app** - You'll see a tabbed interface with 5 main tabs

2. **Input Parameters Tab** (Default tab)
   - All parameters are pre-loaded with the problem values
   - You can adjust any value using:
     - Text fields (type new values)
     - Sliders (drag to adjust interactively)
   - Click **"Calculate"** to compute all parameters

3. **View Results**
   - Click **"Static Analysis"** tab to see detailed calculations
   - Results include:
     - Speed analysis (sync speed, slip)
     - Power analysis (input, output)
     - Power factor
     - Efficiency
     - Torque
     - Losses breakdown

4. **Visualize Performance**
   - Click **"Visualization"** tab
   - See 6 key plots:
     - Speed-Torque curve
     - Efficiency vs Load
     - Power Factor vs Load
     - Current vs Load
     - Power Flow diagram
     - Torque vs Slip

5. **Run Dynamic Simulation**
   - Click **"Dynamic Simulation"** tab
   - Adjust parameters if needed:
     - Supply Voltage (default: 415V)
     - Load Torque (default: 100 Nm)
     - Simulation Time (default: 5s)
   - Select Solver: **RK45** (recommended) or Euler
   - Click **"Start Simulation"**
   - Watch real-time plots of motor startup

6. **Full Analysis**
   - Click **"Full Analysis"** tab
   - Explore 4 sub-tabs:
     - Performance Characteristics
     - Efficiency Analysis
     - Losses Breakdown (pie & bar charts)
     - Circle Diagram

7. **Export Data**
   - Go back to **"Input Parameters"** tab
   - Click **"Export Data"** button
   - Data saved as timestamped text file

## Problem Solution Results

For the given motor test data, the application calculates:

| Parameter | Value |
|-----------|-------|
| Synchronous Speed | 1500 rpm |
| Slip at Full Load | 2.67% |
| Full Load Input Power | 62.5 kW |
| Full Load Power Factor | 0.919 (lagging) |
| Output Power | ~60.5 kW |
| Efficiency | ~96.8% |
| Full Load Torque | ~396 Nm |
| Stator Cu Loss | 1.78 kW |

## Key Features

### Interactive Controls
- **Sliders**: Adjust parameters in real-time
- **Start/Stop/Reset**: Control dynamic simulation
- **Auto-scaling**: Resize window freely

### Multiple Solvers
- **RK45**: High accuracy, adaptive step size (recommended)
- **Euler**: Fast, simple, good for quick analysis

### Comprehensive Plots
- Static characteristics (6 plots)
- Dynamic response (6 plots)
- Full analysis (4 detailed sub-tabs)

## Tips for Best Results

1. **For Accurate Static Analysis**:
   - Use actual measured test values
   - Ensure resistance value is at operating temperature
   - Click "Calculate" after any parameter change

2. **For Dynamic Simulation**:
   - Start with RK45 solver
   - Use 5-10 seconds simulation time for full startup
   - Reduce time to 0.5-1s for quick tests
   - Adjust load torque to see different responses

3. **Performance**:
   - RK45: Better accuracy, slower (~5-10 seconds)
   - Euler: Faster (~1-2 seconds), less accurate
   - For very long simulations (>10s), use Euler

4. **Visualization**:
   - Resize window to see plots better
   - All plots auto-scale to window size
   - Switch between tabs to see different analyses

## Troubleshooting

### "No module named 'tkinter'"
- **Linux**: `sudo apt-get install python3-tk`
- **Mac**: tkinter comes with Python
- **Windows**: tkinter included in standard Python

### "No module named 'numpy'"
- Run: `pip install -r requirements.txt`

### Simulation Takes Too Long
- Reduce simulation time (try 1-2 seconds)
- Use Euler method instead of RK45
- Check that parameters are reasonable

### Plots Not Showing
- Click "Calculate" button first
- Check that all input values are valid numbers
- Try resizing window

### Window Too Small/Large
- Drag window edges to resize
- All content auto-scales
- Minimum recommended: 1200x800 pixels

## Advanced Usage

### Custom Motor Analysis
1. Clear all input fields
2. Enter your motor test data
3. Adjust motor specifications
4. Click "Calculate"
5. View results in all tabs

### Parameter Study
1. Use sliders to vary one parameter
2. Observe changes in real-time
3. Click "Calculate" after each change
4. Export data for comparison

### Teaching/Demonstration
1. Start with default values
2. Show static analysis first
3. Explain each calculation in results
4. Run dynamic simulation
5. Show startup behavior
6. Explore full analysis tabs

## Educational Value

This tool is excellent for:
- Understanding induction motor theory
- Learning two wattmeter method
- Visualizing motor characteristics
- Studying dynamic behavior
- Comparing different operating points
- Energy efficiency analysis

## Next Steps

### Learn More
- Read README_MOTOR_ANALYSIS.md for detailed documentation
- Study the calculated results in Static Analysis tab
- Experiment with different parameters
- Try various load conditions in simulation

### Extend the Application
The code is well-structured and can be extended:
- Add more motor models
- Implement different control strategies
- Add temperature rise calculations
- Include harmonic analysis
- Connect to real motor data

## Support

For issues or questions:
1. Check README_MOTOR_ANALYSIS.md
2. Verify all dependencies installed
3. Ensure input values are reasonable
4. Check Python version (3.7+)

---

**Enjoy exploring motor analysis!** 🎓⚡
