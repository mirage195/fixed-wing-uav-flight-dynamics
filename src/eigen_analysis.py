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
