"""
Three-Phase Induction Motor Power Analysis and Dynamic Simulation
Complete GUI application with real-time simulation and comprehensive analysis
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from scipy.integrate import odeint, solve_ivp
import math
from datetime import datetime


class MotorParameters:
    """Store and calculate motor parameters"""

    def __init__(self):
        # Default parameters from the problem
        self.no_load_w1 = 3.0  # kW
        self.no_load_w2 = -5.0  # kW
        self.full_load_w1 = 39.0  # kW
        self.full_load_w2 = 23.5  # kW
        self.poles = 4
        self.frequency = 50  # Hz
        self.full_load_speed = 1460  # rpm
        self.stator_resistance = 0.13  # Ω per phase
        self.no_load_current = 25.0  # A per line
        self.full_load_current = 92.0  # A per line
        self.line_voltage = 415.0  # V (typical 3-phase)

        # Calculated parameters
        self.sync_speed = 0
        self.slip_full_load = 0
        self.efficiency = 0
        self.power_factor_no_load = 0
        self.power_factor_full_load = 0
        self.output_power = 0
        self.torque_full_load = 0

    def calculate_all(self):
        """Calculate all motor parameters"""
        # Synchronous speed
        self.sync_speed = (120 * self.frequency) / self.poles  # rpm

        # Slip at full load
        self.slip_full_load = (self.sync_speed - self.full_load_speed) / self.sync_speed

        # Power calculations
        # No load
        p_no_load = self.no_load_w1 + self.no_load_w2  # kW

        # Full load
        p_full_load = self.full_load_w1 + self.full_load_w2  # kW

        # Power factor calculations
        # tan(phi) = sqrt(3) * (W1 - W2) / (W1 + W2)
        if abs(p_no_load) > 0.001:
            tan_phi_no_load = math.sqrt(3) * (self.no_load_w1 - self.no_load_w2) / p_no_load
            self.power_factor_no_load = math.cos(math.atan(tan_phi_no_load))
        else:
            self.power_factor_no_load = 0

        if abs(p_full_load) > 0.001:
            tan_phi_full_load = math.sqrt(3) * (self.full_load_w1 - self.full_load_w2) / p_full_load
            self.power_factor_full_load = math.cos(math.atan(tan_phi_full_load))
        else:
            self.power_factor_full_load = 0

        # Stator copper losses (3-phase delta)
        # For delta connection: Phase current = Line current / sqrt(3)
        i_phase_full_load = self.full_load_current / math.sqrt(3)
        stator_cu_loss_full_load = 3 * i_phase_full_load**2 * self.stator_resistance / 1000  # kW

        # No load losses (iron losses + friction & windage)
        i_phase_no_load = self.no_load_current / math.sqrt(3)
        stator_cu_loss_no_load = 3 * i_phase_no_load**2 * self.stator_resistance / 1000  # kW
        no_load_losses = abs(p_no_load) + stator_cu_loss_no_load

        # Output power
        total_losses = no_load_losses + stator_cu_loss_full_load
        self.output_power = p_full_load - total_losses

        # Efficiency
        if p_full_load > 0:
            self.efficiency = (self.output_power / p_full_load) * 100
        else:
            self.efficiency = 0

        # Torque at full load
        omega = 2 * math.pi * self.full_load_speed / 60  # rad/s
        self.torque_full_load = (self.output_power * 1000) / omega  # Nm

        return {
            'sync_speed': self.sync_speed,
            'slip_full_load': self.slip_full_load,
            'p_no_load': p_no_load,
            'p_full_load': p_full_load,
            'pf_no_load': self.power_factor_no_load,
            'pf_full_load': self.power_factor_full_load,
            'stator_cu_loss_full_load': stator_cu_loss_full_load,
            'no_load_losses': no_load_losses,
            'output_power': self.output_power,
            'efficiency': self.efficiency,
            'torque_full_load': self.torque_full_load
        }


class MotorDynamicModel:
    """Dynamic model of induction motor using differential equations"""

    def __init__(self, motor_params):
        self.motor = motor_params

        # Equivalent circuit parameters (estimated)
        self.Rs = motor_params.stator_resistance  # Stator resistance
        self.Rr = 0.15  # Rotor resistance (estimated)
        self.Ls = 0.01  # Stator inductance (H)
        self.Lr = 0.01  # Rotor inductance (H)
        self.Lm = 0.3  # Magnetizing inductance (H)
        self.J = 0.5  # Moment of inertia (kg.m^2)
        self.B = 0.01  # Friction coefficient
        self.P = motor_params.poles  # Number of poles

    def motor_dynamics_rk45(self, t, y, V_supply, T_load, freq):
        """
        Differential equations for motor dynamics (for RK45 solver)
        State variables: [i_sd, i_sq, psi_rd, psi_rq, omega_r]
        """
        i_sd, i_sq, psi_rd, psi_rq, omega_r = y

        # Synchronous speed
        omega_e = 2 * np.pi * freq

        # Slip speed
        omega_slip = omega_e - omega_r * (self.P / 2)

        # Voltage components (assuming balanced supply)
        V_sd = V_supply * np.sqrt(2/3) * np.cos(omega_e * t)
        V_sq = V_supply * np.sqrt(2/3) * np.sin(omega_e * t)

        # Stator equations
        di_sd = (V_sd - self.Rs * i_sd - omega_e * self.Ls * i_sq - (self.Lm / self.Lr) * omega_e * psi_rq) / self.Ls
        di_sq = (V_sq - self.Rs * i_sq + omega_e * self.Ls * i_sd + (self.Lm / self.Lr) * omega_e * psi_rd) / self.Ls

        # Rotor equations
        dpsi_rd = -self.Rr / self.Lr * psi_rd + omega_slip * psi_rq + self.Rr * self.Lm / self.Lr * i_sd
        dpsi_rq = -self.Rr / self.Lr * psi_rq - omega_slip * psi_rd + self.Rr * self.Lm / self.Lr * i_sq

        # Electromagnetic torque
        T_em = (3 / 2) * (self.P / 2) * self.Lm / self.Lr * (psi_rd * i_sq - psi_rq * i_sd)

        # Mechanical equation
        domega_r = (T_em - T_load - self.B * omega_r) / self.J

        return [di_sd, di_sq, dpsi_rd, dpsi_rq, domega_r]

    def motor_dynamics_euler(self, y, t, V_supply, T_load, freq, dt):
        """
        Euler method for motor dynamics
        """
        dydt = self.motor_dynamics_rk45(t, y, V_supply, T_load, freq)
        y_new = [y[i] + dydt[i] * dt for i in range(len(y))]
        return y_new

    def simulate_startup(self, V_supply, T_load, t_end=5.0, method='RK45'):
        """
        Simulate motor startup
        """
        # Initial conditions [i_sd, i_sq, psi_rd, psi_rq, omega_r]
        y0 = [0, 0, 0, 0, 0]

        if method == 'RK45':
            # Time span
            t_span = (0, t_end)
            t_eval = np.linspace(0, t_end, 1000)

            # Solve using RK45
            sol = solve_ivp(
                lambda t, y: self.motor_dynamics_rk45(t, y, V_supply, T_load, self.motor.frequency),
                t_span, y0, method='RK45', t_eval=t_eval, max_step=0.01
            )

            return sol.t, sol.y

        elif method == 'Euler':
            # Euler method
            dt = 0.001
            t = np.arange(0, t_end, dt)
            n_steps = len(t)

            # Initialize arrays
            y = np.zeros((5, n_steps))
            y[:, 0] = y0

            # Integrate
            for i in range(1, n_steps):
                y[:, i] = self.motor_dynamics_euler(
                    y[:, i-1], t[i-1], V_supply, T_load, self.motor.frequency, dt
                )

            return t, y


class MotorAnalysisGUI:
    """Main GUI Application for Motor Analysis"""

    def __init__(self, root):
        self.root = root
        self.root.title("Three-Phase Induction Motor Power Analysis & Dynamic Simulation")
        self.root.geometry("1400x900")

        # Motor parameters
        self.motor = MotorParameters()
        self.dynamic_model = MotorDynamicModel(self.motor)

        # Simulation state
        self.simulation_running = False
        self.simulation_data = None

        # Configure grid weights for auto-scaling
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)

        # Create main notebook (tabs)
        self.notebook = ttk.Notebook(self.root)
        self.notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Create tabs
        self.create_input_tab()
        self.create_static_analysis_tab()
        self.create_dynamic_simulation_tab()
        self.create_visualization_tab()
        self.create_full_analysis_tab()

        # Bind window resize event
        self.root.bind('<Configure>', self.on_window_resize)

        # Initial calculation
        self.calculate_static_parameters()

    def create_input_tab(self):
        """Create input parameters tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Input Parameters")

        # Configure grid
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Main frame with scrollbar
        canvas = tk.Canvas(tab)
        scrollbar = ttk.Scrollbar(tab, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        # Grid layout
        canvas.grid(row=0, column=0, sticky='nsew')
        scrollbar.grid(row=0, column=1, sticky='ns')

        # Input fields
        self.input_vars = {}

        parameters = [
            ("No Load Test", [
                ("W1 (kW)", "no_load_w1", self.motor.no_load_w1),
                ("W2 (kW)", "no_load_w2", self.motor.no_load_w2),
                ("Current (A)", "no_load_current", self.motor.no_load_current),
            ]),
            ("Full Load Test", [
                ("W1 (kW)", "full_load_w1", self.motor.full_load_w1),
                ("W2 (kW)", "full_load_w2", self.motor.full_load_w2),
                ("Current (A)", "full_load_current", self.motor.full_load_current),
                ("Speed (rpm)", "full_load_speed", self.motor.full_load_speed),
            ]),
            ("Motor Specifications", [
                ("Poles", "poles", self.motor.poles),
                ("Frequency (Hz)", "frequency", self.motor.frequency),
                ("Stator Resistance (Ω)", "stator_resistance", self.motor.stator_resistance),
                ("Line Voltage (V)", "line_voltage", self.motor.line_voltage),
            ]),
        ]

        row = 0
        for section_title, params in parameters:
            # Section header
            ttk.Label(scrollable_frame, text=section_title, font=('Arial', 12, 'bold')).grid(
                row=row, column=0, columnspan=3, pady=(10, 5), sticky='w', padx=10
            )
            row += 1

            for label, var_name, default_value in params:
                ttk.Label(scrollable_frame, text=label + ":").grid(
                    row=row, column=0, sticky='e', padx=10, pady=5
                )

                var = tk.DoubleVar(value=default_value)
                self.input_vars[var_name] = var

                entry = ttk.Entry(scrollable_frame, textvariable=var, width=15)
                entry.grid(row=row, column=1, sticky='w', padx=10, pady=5)

                # Add slider for interactive adjustment
                slider = ttk.Scale(
                    scrollable_frame, from_=default_value*0.5, to=default_value*1.5,
                    orient='horizontal', variable=var, length=200
                )
                slider.grid(row=row, column=2, sticky='w', padx=10, pady=5)

                row += 1

        # Control buttons
        button_frame = ttk.Frame(scrollable_frame)
        button_frame.grid(row=row, column=0, columnspan=3, pady=20)

        ttk.Button(button_frame, text="Calculate", command=self.calculate_static_parameters).pack(
            side='left', padx=5
        )
        ttk.Button(button_frame, text="Reset to Default", command=self.reset_parameters).pack(
            side='left', padx=5
        )
        ttk.Button(button_frame, text="Export Data", command=self.export_data).pack(
            side='left', padx=5
        )

    def create_static_analysis_tab(self):
        """Create static analysis results tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Static Analysis")

        # Configure grid
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Results text area
        self.results_text = scrolledtext.ScrolledText(tab, wrap=tk.WORD, width=80, height=30,
                                                       font=('Courier', 10))
        self.results_text.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)

        # Add tags for formatting
        self.results_text.tag_config('header', font=('Courier', 12, 'bold'), foreground='blue')
        self.results_text.tag_config('subheader', font=('Courier', 11, 'bold'), foreground='darkgreen')
        self.results_text.tag_config('value', font=('Courier', 10), foreground='black')

    def create_dynamic_simulation_tab(self):
        """Create dynamic simulation tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Dynamic Simulation")

        # Configure grid
        tab.grid_rowconfigure(1, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Control panel
        control_frame = ttk.LabelFrame(tab, text="Simulation Controls", padding=10)
        control_frame.grid(row=0, column=0, sticky='ew', padx=10, pady=10)

        # Simulation parameters
        ttk.Label(control_frame, text="Supply Voltage (V):").grid(row=0, column=0, sticky='e', padx=5, pady=5)
        self.sim_voltage_var = tk.DoubleVar(value=415)
        ttk.Entry(control_frame, textvariable=self.sim_voltage_var, width=10).grid(
            row=0, column=1, sticky='w', padx=5, pady=5
        )

        ttk.Label(control_frame, text="Load Torque (Nm):").grid(row=0, column=2, sticky='e', padx=5, pady=5)
        self.sim_torque_var = tk.DoubleVar(value=100)
        ttk.Entry(control_frame, textvariable=self.sim_torque_var, width=10).grid(
            row=0, column=3, sticky='w', padx=5, pady=5
        )

        ttk.Label(control_frame, text="Simulation Time (s):").grid(row=0, column=4, sticky='e', padx=5, pady=5)
        self.sim_time_var = tk.DoubleVar(value=5.0)
        ttk.Entry(control_frame, textvariable=self.sim_time_var, width=10).grid(
            row=0, column=5, sticky='w', padx=5, pady=5
        )

        ttk.Label(control_frame, text="Solver Method:").grid(row=1, column=0, sticky='e', padx=5, pady=5)
        self.solver_var = tk.StringVar(value='RK45')
        solver_combo = ttk.Combobox(control_frame, textvariable=self.solver_var,
                                     values=['RK45', 'Euler'], width=10, state='readonly')
        solver_combo.grid(row=1, column=1, sticky='w', padx=5, pady=5)

        # Control buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=1, column=2, columnspan=4, pady=5)

        self.start_btn = ttk.Button(button_frame, text="Start Simulation", command=self.start_simulation)
        self.start_btn.pack(side='left', padx=5)

        self.stop_btn = ttk.Button(button_frame, text="Stop Simulation", command=self.stop_simulation, state='disabled')
        self.stop_btn.pack(side='left', padx=5)

        ttk.Button(button_frame, text="Reset", command=self.reset_simulation).pack(side='left', padx=5)

        # Progress bar
        self.progress_var = tk.DoubleVar()
        self.progress_bar = ttk.Progressbar(control_frame, variable=self.progress_var, maximum=100)
        self.progress_bar.grid(row=2, column=0, columnspan=6, sticky='ew', padx=5, pady=5)

        # Plot area
        self.sim_plot_frame = ttk.Frame(tab)
        self.sim_plot_frame.grid(row=1, column=0, sticky='nsew', padx=10, pady=10)
        self.sim_plot_frame.grid_rowconfigure(0, weight=1)
        self.sim_plot_frame.grid_columnconfigure(0, weight=1)

    def create_visualization_tab(self):
        """Create visualization tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Visualization")

        # Configure grid
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Plot frame
        self.viz_plot_frame = ttk.Frame(tab)
        self.viz_plot_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
        self.viz_plot_frame.grid_rowconfigure(0, weight=1)
        self.viz_plot_frame.grid_columnconfigure(0, weight=1)

        # Create initial plots
        self.create_static_plots()

    def create_full_analysis_tab(self):
        """Create comprehensive analysis tab"""
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text="Full Analysis")

        # Configure grid
        tab.grid_rowconfigure(0, weight=1)
        tab.grid_columnconfigure(0, weight=1)

        # Create notebook for sub-sections
        sub_notebook = ttk.Notebook(tab)
        sub_notebook.grid(row=0, column=0, sticky='nsew', padx=5, pady=5)

        # Performance characteristics
        perf_tab = ttk.Frame(sub_notebook)
        sub_notebook.add(perf_tab, text="Performance Characteristics")
        self.create_performance_analysis(perf_tab)

        # Efficiency analysis
        eff_tab = ttk.Frame(sub_notebook)
        sub_notebook.add(eff_tab, text="Efficiency Analysis")
        self.create_efficiency_analysis(eff_tab)

        # Losses breakdown
        loss_tab = ttk.Frame(sub_notebook)
        sub_notebook.add(loss_tab, text="Losses Breakdown")
        self.create_losses_analysis(loss_tab)

        # Circle diagram
        circle_tab = ttk.Frame(sub_notebook)
        sub_notebook.add(circle_tab, text="Circle Diagram")
        self.create_circle_diagram(circle_tab)

    def create_performance_analysis(self, parent):
        """Create performance characteristics plot"""
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=1)

        self.perf_frame = ttk.Frame(parent)
        self.perf_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
        self.perf_frame.grid_rowconfigure(0, weight=1)
        self.perf_frame.grid_columnconfigure(0, weight=1)

    def create_efficiency_analysis(self, parent):
        """Create efficiency analysis plot"""
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=1)

        self.eff_frame = ttk.Frame(parent)
        self.eff_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
        self.eff_frame.grid_rowconfigure(0, weight=1)
        self.eff_frame.grid_columnconfigure(0, weight=1)

    def create_losses_analysis(self, parent):
        """Create losses breakdown visualization"""
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=1)

        self.loss_frame = ttk.Frame(parent)
        self.loss_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
        self.loss_frame.grid_rowconfigure(0, weight=1)
        self.loss_frame.grid_columnconfigure(0, weight=1)

    def create_circle_diagram(self, parent):
        """Create circle diagram"""
        parent.grid_rowconfigure(0, weight=1)
        parent.grid_columnconfigure(0, weight=1)

        self.circle_frame = ttk.Frame(parent)
        self.circle_frame.grid(row=0, column=0, sticky='nsew', padx=10, pady=10)
        self.circle_frame.grid_rowconfigure(0, weight=1)
        self.circle_frame.grid_columnconfigure(0, weight=1)

    def update_motor_from_inputs(self):
        """Update motor parameters from input fields"""
        self.motor.no_load_w1 = self.input_vars['no_load_w1'].get()
        self.motor.no_load_w2 = self.input_vars['no_load_w2'].get()
        self.motor.no_load_current = self.input_vars['no_load_current'].get()
        self.motor.full_load_w1 = self.input_vars['full_load_w1'].get()
        self.motor.full_load_w2 = self.input_vars['full_load_w2'].get()
        self.motor.full_load_current = self.input_vars['full_load_current'].get()
        self.motor.full_load_speed = self.input_vars['full_load_speed'].get()
        self.motor.poles = int(self.input_vars['poles'].get())
        self.motor.frequency = self.input_vars['frequency'].get()
        self.motor.stator_resistance = self.input_vars['stator_resistance'].get()
        self.motor.line_voltage = self.input_vars['line_voltage'].get()

    def calculate_static_parameters(self):
        """Calculate and display static parameters"""
        # Update motor from inputs
        self.update_motor_from_inputs()

        # Calculate parameters
        results = self.motor.calculate_all()

        # Display results
        self.results_text.delete('1.0', tk.END)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        output = f"""
{'='*80}
THREE-PHASE INDUCTION MOTOR ANALYSIS REPORT
{'='*80}
Generated: {timestamp}

{'='*80}
INPUT PARAMETERS
{'='*80}

No Load Test:
  W1 = {self.motor.no_load_w1:.2f} kW
  W2 = {self.motor.no_load_w2:.2f} kW
  Current = {self.motor.no_load_current:.2f} A

Full Load Test:
  W1 = {self.motor.full_load_w1:.2f} kW
  W2 = {self.motor.full_load_w2:.2f} kW
  Current = {self.motor.full_load_current:.2f} A
  Speed = {self.motor.full_load_speed:.0f} rpm

Motor Specifications:
  Number of Poles = {self.motor.poles}
  Supply Frequency = {self.motor.frequency} Hz
  Stator Resistance = {self.motor.stator_resistance} Ω/phase
  Line Voltage = {self.motor.line_voltage} V

{'='*80}
CALCULATED RESULTS
{'='*80}

Speed Analysis:
  Synchronous Speed = {results['sync_speed']:.2f} rpm
  Full Load Speed = {self.motor.full_load_speed:.2f} rpm
  Slip at Full Load = {results['slip_full_load']*100:.3f} %

Power Analysis:
  No Load Input Power = {results['p_no_load']:.3f} kW
  Full Load Input Power = {results['p_full_load']:.3f} kW
  Output Power = {results['output_power']:.3f} kW

Power Factor:
  No Load Power Factor = {results['pf_no_load']:.4f} {'(leading)' if results['p_no_load'] < 0 else '(lagging)'}
  Full Load Power Factor = {results['pf_full_load']:.4f} {'(leading)' if results['p_full_load'] < 0 else '(lagging)'}

Losses:
  Stator Cu Loss (Full Load) = {results['stator_cu_loss_full_load']:.3f} kW
  No Load Losses (Iron + Friction) = {results['no_load_losses']:.3f} kW
  Total Losses = {results['stator_cu_loss_full_load'] + results['no_load_losses']:.3f} kW

Performance:
  Efficiency = {results['efficiency']:.2f} %
  Torque at Full Load = {results['torque_full_load']:.2f} Nm

{'='*80}
ADDITIONAL CALCULATIONS
{'='*80}

Angular Velocities:
  Synchronous (ωs) = {2*np.pi*results['sync_speed']/60:.3f} rad/s
  Full Load (ωr) = {2*np.pi*self.motor.full_load_speed/60:.3f} rad/s

Phase Currents (Delta Connection):
  No Load Phase Current = {self.motor.no_load_current/np.sqrt(3):.2f} A
  Full Load Phase Current = {self.motor.full_load_current/np.sqrt(3):.2f} A

Apparent Power:
  Full Load = {np.sqrt(3)*self.motor.line_voltage*self.motor.full_load_current/1000:.2f} kVA

{'='*80}
"""

        self.results_text.insert('1.0', output)

        # Update visualizations
        self.create_static_plots()
        self.update_full_analysis()

        messagebox.showinfo("Calculation Complete", "Static analysis completed successfully!")

    def create_static_plots(self):
        """Create static analysis plots"""
        # Clear existing plots
        for widget in self.viz_plot_frame.winfo_children():
            widget.destroy()

        # Create figure
        fig = Figure(figsize=(12, 8), dpi=100)

        # Calculate over a range of loads
        loads = np.linspace(0, 1.2, 50)
        speeds = []
        torques = []
        efficiencies = []
        power_factors = []
        currents = []

        for load in loads:
            # Simple linear approximation for demonstration
            slip = self.motor.slip_full_load * load
            speed = self.motor.sync_speed * (1 - slip)
            speeds.append(speed)

            torque = self.motor.torque_full_load * load
            torques.append(torque)

            if load < 0.1:
                eff = 0
            else:
                eff = self.motor.efficiency * (load / (0.5 + 0.5 * load))
            efficiencies.append(min(eff, 100))

            pf = self.motor.power_factor_full_load * (0.5 + 0.5 * load)
            power_factors.append(min(pf, 1.0))

            current = self.motor.no_load_current + (self.motor.full_load_current - self.motor.no_load_current) * load
            currents.append(current)

        # Plot 1: Speed-Torque Curve
        ax1 = fig.add_subplot(2, 3, 1)
        ax1.plot(torques, speeds, 'b-', linewidth=2)
        ax1.axhline(y=self.motor.sync_speed, color='r', linestyle='--', label='Sync Speed')
        ax1.axvline(x=self.motor.torque_full_load, color='g', linestyle='--', label='Full Load')
        ax1.set_xlabel('Torque (Nm)')
        ax1.set_ylabel('Speed (rpm)')
        ax1.set_title('Speed-Torque Characteristic')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Plot 2: Efficiency vs Load
        ax2 = fig.add_subplot(2, 3, 2)
        ax2.plot(loads * 100, efficiencies, 'g-', linewidth=2)
        ax2.set_xlabel('Load (%)')
        ax2.set_ylabel('Efficiency (%)')
        ax2.set_title('Efficiency vs Load')
        ax2.grid(True, alpha=0.3)
        ax2.set_ylim([0, 105])

        # Plot 3: Power Factor vs Load
        ax3 = fig.add_subplot(2, 3, 3)
        ax3.plot(loads * 100, power_factors, 'r-', linewidth=2)
        ax3.set_xlabel('Load (%)')
        ax3.set_ylabel('Power Factor')
        ax3.set_title('Power Factor vs Load')
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim([0, 1.1])

        # Plot 4: Current vs Load
        ax4 = fig.add_subplot(2, 3, 4)
        ax4.plot(loads * 100, currents, 'm-', linewidth=2)
        ax4.set_xlabel('Load (%)')
        ax4.set_ylabel('Current (A)')
        ax4.set_title('Current vs Load')
        ax4.grid(True, alpha=0.3)

        # Plot 5: Power Flow Diagram
        ax5 = fig.add_subplot(2, 3, 5)
        results = self.motor.calculate_all()

        p_in = results['p_full_load']
        p_stator_loss = results['stator_cu_loss_full_load']
        p_no_load = results['no_load_losses']
        p_out = results['output_power']

        powers = [p_in, p_in - p_stator_loss, p_out]
        labels = ['Input', 'Air Gap', 'Output']
        x_pos = np.arange(len(labels))

        ax5.bar(x_pos, powers, color=['blue', 'orange', 'green'], alpha=0.7)
        ax5.set_xticks(x_pos)
        ax5.set_xticklabels(labels)
        ax5.set_ylabel('Power (kW)')
        ax5.set_title('Power Flow')
        ax5.grid(True, alpha=0.3, axis='y')

        # Plot 6: Slip vs Torque
        ax6 = fig.add_subplot(2, 3, 6)
        slips = [self.motor.slip_full_load * load for load in loads]
        ax6.plot(np.array(slips) * 100, torques, 'c-', linewidth=2)
        ax6.set_xlabel('Slip (%)')
        ax6.set_ylabel('Torque (Nm)')
        ax6.set_title('Torque vs Slip')
        ax6.grid(True, alpha=0.3)

        fig.tight_layout()

        # Embed in tkinter
        canvas = FigureCanvasTkAgg(fig, master=self.viz_plot_frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def start_simulation(self):
        """Start dynamic simulation"""
        self.simulation_running = True
        self.start_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.progress_var.set(0)

        try:
            # Get simulation parameters
            V_supply = self.sim_voltage_var.get() / np.sqrt(3)  # Phase voltage
            T_load = self.sim_torque_var.get()
            t_end = self.sim_time_var.get()
            method = self.solver_var.get()

            # Update progress
            self.progress_var.set(20)
            self.root.update()

            # Run simulation
            t, y = self.dynamic_model.simulate_startup(V_supply, T_load, t_end, method)

            # Update progress
            self.progress_var.set(60)
            self.root.update()

            # Store results
            self.simulation_data = {'t': t, 'y': y, 'method': method}

            # Plot results
            self.plot_simulation_results()

            # Update progress
            self.progress_var.set(100)

            messagebox.showinfo("Simulation Complete",
                              f"Dynamic simulation completed using {method} method!")

        except Exception as e:
            messagebox.showerror("Simulation Error", f"Error during simulation:\n{str(e)}")

        finally:
            self.simulation_running = False
            self.start_btn.config(state='normal')
            self.stop_btn.config(state='disabled')

    def stop_simulation(self):
        """Stop running simulation"""
        self.simulation_running = False
        self.start_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        messagebox.showinfo("Simulation Stopped", "Simulation stopped by user.")

    def reset_simulation(self):
        """Reset simulation"""
        self.simulation_data = None
        self.progress_var.set(0)

        # Clear plots
        for widget in self.sim_plot_frame.winfo_children():
            widget.destroy()

        messagebox.showinfo("Reset", "Simulation reset successfully.")

    def plot_simulation_results(self):
        """Plot dynamic simulation results"""
        if self.simulation_data is None:
            return

        # Clear existing plots
        for widget in self.sim_plot_frame.winfo_children():
            widget.destroy()

        t = self.simulation_data['t']
        y = self.simulation_data['y']

        # Create figure
        fig = Figure(figsize=(12, 8), dpi=100)

        # Plot 1: Speed
        ax1 = fig.add_subplot(2, 3, 1)
        speed_rpm = y[4] * 60 / (2 * np.pi) * (2 / self.motor.poles)
        ax1.plot(t, speed_rpm, 'b-', linewidth=2)
        ax1.axhline(y=self.motor.sync_speed, color='r', linestyle='--', label='Sync Speed')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Speed (rpm)')
        ax1.set_title('Motor Speed vs Time')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        # Plot 2: Stator Current (magnitude)
        ax2 = fig.add_subplot(2, 3, 2)
        i_s = np.sqrt(y[0]**2 + y[1]**2)
        ax2.plot(t, i_s, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Current (A)')
        ax2.set_title('Stator Current Magnitude')
        ax2.grid(True, alpha=0.3)

        # Plot 3: Rotor Flux
        ax3 = fig.add_subplot(2, 3, 3)
        psi_r = np.sqrt(y[2]**2 + y[3]**2)
        ax3.plot(t, psi_r, 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Flux (Wb)')
        ax3.set_title('Rotor Flux Magnitude')
        ax3.grid(True, alpha=0.3)

        # Plot 4: Electromagnetic Torque
        ax4 = fig.add_subplot(2, 3, 4)
        T_em = (3/2) * (self.motor.poles/2) * self.dynamic_model.Lm / self.dynamic_model.Lr * \
               (y[2] * y[1] - y[3] * y[0])
        ax4.plot(t, T_em, 'm-', linewidth=2)
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Torque (Nm)')
        ax4.set_title('Electromagnetic Torque')
        ax4.grid(True, alpha=0.3)

        # Plot 5: d-q Currents
        ax5 = fig.add_subplot(2, 3, 5)
        ax5.plot(t, y[0], 'b-', linewidth=2, label='i_sd')
        ax5.plot(t, y[1], 'r-', linewidth=2, label='i_sq')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Current (A)')
        ax5.set_title('d-q Axis Stator Currents')
        ax5.grid(True, alpha=0.3)
        ax5.legend()

        # Plot 6: d-q Fluxes
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.plot(t, y[2], 'b-', linewidth=2, label='ψ_rd')
        ax6.plot(t, y[3], 'r-', linewidth=2, label='ψ_rq')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Flux (Wb)')
        ax6.set_title('d-q Axis Rotor Fluxes')
        ax6.grid(True, alpha=0.3)
        ax6.legend()

        fig.tight_layout()

        # Embed in tkinter
        canvas = FigureCanvasTkAgg(fig, master=self.sim_plot_frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def update_full_analysis(self):
        """Update full analysis plots"""
        # Performance characteristics
        self.update_performance_plot()

        # Efficiency analysis
        self.update_efficiency_plot()

        # Losses breakdown
        self.update_losses_plot()

        # Circle diagram
        self.update_circle_diagram()

    def update_performance_plot(self):
        """Update performance characteristics"""
        for widget in self.perf_frame.winfo_children():
            widget.destroy()

        fig = Figure(figsize=(10, 6), dpi=100)

        # Generate load range
        loads = np.linspace(0, 1.5, 100)

        # Calculate parameters at different loads
        power_out = []
        power_in = []
        torque = []
        speed = []

        for load in loads:
            slip = self.motor.slip_full_load * load
            n = self.motor.sync_speed * (1 - slip)
            speed.append(n)

            T = self.motor.torque_full_load * load
            torque.append(T)

            P_out = T * 2 * np.pi * n / 60 / 1000  # kW
            power_out.append(P_out)

            results = self.motor.calculate_all()
            if load > 0:
                P_in = results['p_full_load'] * load / 1.0
            else:
                P_in = results['p_no_load']
            power_in.append(P_in)

        ax = fig.add_subplot(111)
        ax2 = ax.twinx()

        l1 = ax.plot(loads * 100, power_out, 'b-', linewidth=2, label='Output Power')
        l2 = ax.plot(loads * 100, power_in, 'g-', linewidth=2, label='Input Power')
        l3 = ax2.plot(loads * 100, torque, 'r-', linewidth=2, label='Torque')

        ax.set_xlabel('Load (%)', fontsize=12)
        ax.set_ylabel('Power (kW)', fontsize=12)
        ax2.set_ylabel('Torque (Nm)', fontsize=12)
        ax.set_title('Motor Performance Characteristics', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Combine legends
        lns = l1 + l2 + l3
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='upper left')

        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=self.perf_frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def update_efficiency_plot(self):
        """Update efficiency analysis"""
        for widget in self.eff_frame.winfo_children():
            widget.destroy()

        fig = Figure(figsize=(10, 6), dpi=100)

        loads = np.linspace(0.1, 1.5, 100)
        efficiency = []
        power_factor = []

        for load in loads:
            # Efficiency curve (approximation)
            if load < 0.25:
                eff = self.motor.efficiency * 0.5 * load / 0.25
            elif load <= 1.0:
                eff = self.motor.efficiency * (0.5 + 0.5 * load)
            else:
                eff = self.motor.efficiency * (1.0 - 0.1 * (load - 1.0))
            efficiency.append(min(eff, 100))

            # Power factor curve
            pf = self.motor.power_factor_full_load * min(load, 1.0)
            power_factor.append(min(pf, 1.0))

        ax = fig.add_subplot(111)
        ax2 = ax.twinx()

        l1 = ax.plot(loads * 100, efficiency, 'b-', linewidth=2, label='Efficiency')
        l2 = ax2.plot(loads * 100, power_factor, 'r-', linewidth=2, label='Power Factor')

        ax.set_xlabel('Load (%)', fontsize=12)
        ax.set_ylabel('Efficiency (%)', fontsize=12, color='b')
        ax2.set_ylabel('Power Factor', fontsize=12, color='r')
        ax.set_title('Efficiency and Power Factor Analysis', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.tick_params(axis='y', labelcolor='b')
        ax2.tick_params(axis='y', labelcolor='r')

        # Combine legends
        lns = l1 + l2
        labs = [l.get_label() for l in lns]
        ax.legend(lns, labs, loc='lower right')

        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=self.eff_frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def update_losses_plot(self):
        """Update losses breakdown"""
        for widget in self.loss_frame.winfo_children():
            widget.destroy()

        fig = Figure(figsize=(10, 6), dpi=100)

        results = self.motor.calculate_all()

        # Calculate individual losses
        stator_cu_loss = results['stator_cu_loss_full_load']
        no_load_losses = results['no_load_losses']

        # Estimate breakdown
        iron_loss = no_load_losses * 0.7  # Approximate
        friction_windage = no_load_losses * 0.3
        rotor_cu_loss = results['p_full_load'] - results['output_power'] - stator_cu_loss - no_load_losses

        # Pie chart
        ax1 = fig.add_subplot(1, 2, 1)
        losses = [stator_cu_loss, rotor_cu_loss, iron_loss, friction_windage]
        labels = ['Stator Cu Loss', 'Rotor Cu Loss', 'Iron Loss', 'Friction & Windage']
        colors = ['#ff9999', '#66b3ff', '#99ff99', '#ffcc99']
        explode = (0.1, 0, 0, 0)

        ax1.pie(losses, explode=explode, labels=labels, colors=colors, autopct='%1.1f%%',
                shadow=True, startangle=90)
        ax1.set_title('Losses Distribution at Full Load', fontsize=14, fontweight='bold')

        # Bar chart
        ax2 = fig.add_subplot(1, 2, 2)
        x_pos = np.arange(len(labels))
        ax2.bar(x_pos, losses, color=colors, alpha=0.7)
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(labels, rotation=45, ha='right')
        ax2.set_ylabel('Power Loss (kW)', fontsize=12)
        ax2.set_title('Losses Breakdown', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3, axis='y')

        # Add values on bars
        for i, v in enumerate(losses):
            ax2.text(i, v + 0.1, f'{v:.2f} kW', ha='center', va='bottom', fontweight='bold')

        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=self.loss_frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def update_circle_diagram(self):
        """Update circle diagram"""
        for widget in self.circle_frame.winfo_children():
            widget.destroy()

        fig = Figure(figsize=(10, 8), dpi=100)
        ax = fig.add_subplot(111)

        # Generate circle diagram data
        # This is a simplified representation
        theta = np.linspace(0, np.pi, 100)

        # No load point
        I_nl = self.motor.no_load_current
        pf_nl = self.motor.power_factor_no_load
        phi_nl = np.arccos(pf_nl)

        # Full load point
        I_fl = self.motor.full_load_current
        pf_fl = self.motor.power_factor_full_load
        phi_fl = np.arccos(pf_fl)

        # Circle approximation
        center_x = I_fl * pf_fl / 2
        center_y = I_fl * np.sin(phi_fl) / 2
        radius = I_fl / 2

        x_circle = center_x + radius * np.cos(theta)
        y_circle = center_y + radius * np.sin(theta)

        # Plot circle
        ax.plot(x_circle, y_circle, 'b-', linewidth=2, label='Operating Locus')

        # Plot operating points
        ax.plot(I_nl * pf_nl, I_nl * np.sin(phi_nl), 'go', markersize=10, label='No Load')
        ax.plot(I_fl * pf_fl, I_fl * np.sin(phi_fl), 'ro', markersize=10, label='Full Load')

        # Add reference lines
        ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
        ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5)

        # Power factor lines
        for pf in [0.6, 0.7, 0.8, 0.9, 1.0]:
            phi = np.arccos(pf)
            x_line = np.linspace(0, I_fl * 1.2, 10)
            y_line = x_line * np.tan(phi)
            ax.plot(x_line, y_line, 'k--', linewidth=0.5, alpha=0.3)
            ax.text(x_line[-1], y_line[-1], f'pf={pf}', fontsize=8, alpha=0.5)

        ax.set_xlabel('Active Component of Current (A)', fontsize=12)
        ax.set_ylabel('Reactive Component of Current (A)', fontsize=12)
        ax.set_title('Circle Diagram', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_aspect('equal', adjustable='box')

        fig.tight_layout()

        canvas = FigureCanvasTkAgg(fig, master=self.circle_frame)
        canvas.draw()
        canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew')

    def reset_parameters(self):
        """Reset all parameters to default values"""
        # Reset motor parameters
        self.motor = MotorParameters()

        # Update input fields
        for var_name, var in self.input_vars.items():
            var.set(getattr(self.motor, var_name))

        # Recalculate
        self.calculate_static_parameters()

        messagebox.showinfo("Reset", "All parameters reset to default values.")

    def export_data(self):
        """Export analysis data to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"motor_analysis_{timestamp}.txt"

        try:
            with open(filename, 'w') as f:
                # Write all results
                f.write(self.results_text.get('1.0', tk.END))

                # Add simulation data if available
                if self.simulation_data is not None:
                    f.write("\n\n")
                    f.write("="*80 + "\n")
                    f.write("DYNAMIC SIMULATION DATA\n")
                    f.write("="*80 + "\n")
                    f.write(f"Method: {self.simulation_data['method']}\n")
                    f.write(f"Time points: {len(self.simulation_data['t'])}\n")
                    f.write("\nTime (s), Speed (rpm), Current (A), Flux (Wb), Torque (Nm)\n")

                    t = self.simulation_data['t']
                    y = self.simulation_data['y']

                    for i in range(len(t)):
                        speed_rpm = y[4][i] * 60 / (2 * np.pi) * (2 / self.motor.poles)
                        i_s = np.sqrt(y[0][i]**2 + y[1][i]**2)
                        psi_r = np.sqrt(y[2][i]**2 + y[3][i]**2)
                        T_em = (3/2) * (self.motor.poles/2) * self.dynamic_model.Lm / \
                               self.dynamic_model.Lr * (y[2][i] * y[1][i] - y[3][i] * y[0][i])

                        f.write(f"{t[i]:.6f}, {speed_rpm:.2f}, {i_s:.2f}, {psi_r:.6f}, {T_em:.2f}\n")

            messagebox.showinfo("Export Successful", f"Data exported to {filename}")

        except Exception as e:
            messagebox.showerror("Export Error", f"Error exporting data:\n{str(e)}")

    def on_window_resize(self, event):
        """Handle window resize events for auto-scaling"""
        # This is called automatically when window is resized
        # The grid layout with weight configurations handles the scaling
        pass


def main():
    """Main application entry point"""
    root = tk.Tk()
    app = MotorAnalysisGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
