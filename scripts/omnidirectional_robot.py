import numpy as np

# Parámetros
S1, S2, S3 = 0, 1, 0
alpha_1, alpha_2, alpha_3 = 30, 150, 270  # en grados

# Conversión a radianes
a1 = np.deg2rad(alpha_1)
a2 = np.deg2rad(alpha_2)
a3 = np.deg2rad(alpha_3)

# Cálculo de las contribuciones X e Y de cada rueda
Wheel_1_X = np.cos(a1 + np.pi/2)
Wheel_2_X = np.cos(a2 + np.pi/2)
Wheel_3_X = np.cos(a3 + np.pi/2)

Wheel_1_Y = np.sin(a1 + np.pi/2)
Wheel_2_Y = np.sin(a2 + np.pi/2)
Wheel_3_Y = np.sin(a3 + np.pi/2)

# Velocidades lineales del cuerpo
Linear_Velocity_X = Wheel_1_X + Wheel_2_X + Wheel_3_X
Linear_Velocity_Y = Wheel_1_Y + Wheel_2_Y + Wheel_3_Y

# Matriz de cinemática directa M
M = np.array([
    [Wheel_1_X, Wheel_2_X, Wheel_3_X],
    [Wheel_1_Y, Wheel_2_Y, Wheel_3_Y],
    [       1.0,        1.0,        1.0]
])

# Vector de velocidades de rueda
S = np.array([S1, S2, S3])

# Velocidad resultante usando multiplicación de matrices
v = M @ S  # equivalente a M*[S1; S2; S3] en MATLAB

print("Wheel contributions X:", [Wheel_1_X, Wheel_2_X, Wheel_3_X])
print("Wheel contributions Y:", [Wheel_1_Y, Wheel_2_Y, Wheel_3_Y])
print(f"\nLinear_Velocity_X = {Linear_Velocity_X:.3f}")
print(f"Linear_Velocity_Y = {Linear_Velocity_Y:.3f}")

print("\nM (cinemática directa):")
print(M)

print("\nv = M @ S =")
print(v)  # [vx, vy, ω], donde ω = S1+S2+S3
