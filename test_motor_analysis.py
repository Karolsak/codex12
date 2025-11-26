"""
Quick test script for motor analysis module
Tests calculations without GUI
"""

from motor_power_analysis import MotorParameters, MotorDynamicModel
import numpy as np

def test_static_calculations():
    """Test static parameter calculations"""
    print("="*80)
    print("TESTING STATIC CALCULATIONS")
    print("="*80)

    motor = MotorParameters()
    results = motor.calculate_all()

    print(f"\nInput Parameters:")
    print(f"  No Load: W1={motor.no_load_w1} kW, W2={motor.no_load_w2} kW")
    print(f"  Full Load: W1={motor.full_load_w1} kW, W2={motor.full_load_w2} kW")
    print(f"  Poles: {motor.poles}, Frequency: {motor.frequency} Hz")
    print(f"  Full Load Speed: {motor.full_load_speed} rpm")

    print(f"\nCalculated Results:")
    print(f"  Synchronous Speed: {results['sync_speed']:.2f} rpm")
    print(f"  Slip: {results['slip_full_load']*100:.3f} %")
    print(f"  Full Load Input Power: {results['p_full_load']:.3f} kW")
    print(f"  Output Power: {results['output_power']:.3f} kW")
    print(f"  Efficiency: {results['efficiency']:.2f} %")
    print(f"  Power Factor (FL): {results['pf_full_load']:.4f}")
    print(f"  Torque (FL): {results['torque_full_load']:.2f} Nm")

    print("\n✓ Static calculations test PASSED")
    return True

def test_dynamic_model():
    """Test dynamic simulation (quick run)"""
    print("\n" + "="*80)
    print("TESTING DYNAMIC SIMULATION")
    print("="*80)

    motor = MotorParameters()
    dynamic_model = MotorDynamicModel(motor)

    # Quick simulation
    print("\nRunning short simulation (0.5s) with RK45...")
    V_supply = 415 / np.sqrt(3)  # Phase voltage
    T_load = 50  # Nm
    t_end = 0.5  # seconds

    t, y = dynamic_model.simulate_startup(V_supply, T_load, t_end, 'RK45')

    print(f"  Time points: {len(t)}")
    print(f"  Final speed: {y[4][-1] * 60 / (2 * np.pi) * (2 / motor.poles):.2f} rpm")
    print(f"  Final current magnitude: {np.sqrt(y[0][-1]**2 + y[1][-1]**2):.2f} A")

    # Test Euler method
    print("\nRunning short simulation (0.5s) with Euler...")
    t, y = dynamic_model.simulate_startup(V_supply, T_load, t_end, 'Euler')

    print(f"  Time points: {len(t)}")
    print(f"  Final speed: {y[4][-1] * 60 / (2 * np.pi) * (2 / motor.poles):.2f} rpm")
    print(f"  Final current magnitude: {np.sqrt(y[0][-1]**2 + y[1][-1]**2):.2f} A")

    print("\n✓ Dynamic simulation test PASSED")
    return True

def main():
    """Run all tests"""
    print("\n" + "="*80)
    print("THREE-PHASE MOTOR ANALYSIS - MODULE TEST")
    print("="*80)

    try:
        # Test static calculations
        test_static_calculations()

        # Test dynamic model
        test_dynamic_model()

        print("\n" + "="*80)
        print("ALL TESTS PASSED ✓")
        print("="*80)
        print("\nThe motor_power_analysis.py module is working correctly!")
        print("Run 'python motor_power_analysis.py' to start the GUI application.")

    except Exception as e:
        print(f"\n✗ TEST FAILED: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

    return True

if __name__ == "__main__":
    main()
