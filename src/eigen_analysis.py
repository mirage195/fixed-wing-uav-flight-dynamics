import numpy as np
import matplotlib.pyplot as plt

def analyze_longitudinal_modes(A):

    eigvals, eigvecs = np.linalg.eig(A)

    print("\nEigenvalues:")
    print(eigvals)

    print("\nMode Classification:")

    for val in eigvals:
        if abs(val.imag) > 0:
            if abs(val.real) > 1:
                print("Short Period Mode:", val)
            else:
                print("Phugoid Mode:", val)
        else:
            print("Real Mode:", val)

    return eigvals


def plot_eigenvalues(A):

    eigvals, _ = np.linalg.eig(A)

    plt.figure()

    plt.scatter(eigvals.real, eigvals.imag)

    plt.axvline(0)

    plt.xlabel("Real Part")
    plt.ylabel("Imaginary Part")
    plt.title("Longitudinal Modes")

    plt.grid()

    plt.show()


def analyze_lateral_modes(A):
    eigvals = np.linalg.eigvals(A)

    print("\nEigenvalues:")
    print(eigvals)

    print("\nMode Classification:")

    real_modes = []
    complex_modes = []

    # Separate real and complex
    for val in eigvals:
        if abs(val.imag) < 1e-6:
            real_modes.append(val.real)
        else:
            complex_modes.append(val)

    # -------------------------
    # REAL MODES
    # -------------------------
    if len(real_modes) >= 2:
        real_modes_sorted = sorted(real_modes)

        roll_mode = min(real_modes_sorted, key=lambda x: abs(x))  # fastest decay usually
        spiral_mode = max(real_modes_sorted, key=lambda x: x)     # closest to zero

        print(f"Roll Mode: {roll_mode}")
        print(f"Spiral Mode: {spiral_mode}")
    else:
        for val in real_modes:
            print(f"Real Mode: {val}")

    # -------------------------
    # DUTCH ROLL (complex pair)
    # -------------------------
    if complex_modes:
        for val in complex_modes:
            wn = np.sqrt(val.real**2 + val.imag**2)
            zeta = -val.real / wn if wn != 0 else 0

            print(f"Dutch Roll Mode: {val}")
            print(f"  Natural Frequency (wn): {wn:.4f}")
            print(f"  Damping Ratio (zeta): {zeta:.4f}")