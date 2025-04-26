#!/usr/bin/env python3
import numpy as np

def construir_matriz_cinematica(alphas_deg):
    """
    Construye la matriz M de cinemática directa (3×3) para un robot
    omnidireccional de tres ruedas, dadas sus orientaciones en grados.
    """
    # Convertir grados a radianes
    alphas_rad = np.deg2rad(alphas_deg)
    # Montar M
    M = np.array([
        [np.cos(alphas_rad[0] + np.pi/2),
         np.cos(alphas_rad[1] + np.pi/2),
         np.cos(alphas_rad[2] + np.pi/2)],
        [np.sin(alphas_rad[0] + np.pi/2),
         np.sin(alphas_rad[1] + np.pi/2),
         np.sin(alphas_rad[2] + np.pi/2)],
        [1.0, 1.0, 1.0]
    ])
    return M

def main():
    # 1) Definir ángulos de las ruedas (en grados)
    alpha_1, alpha_2, alpha_3 = 30, 150, 270

    # 2) Construir M
    M = construir_matriz_cinematica([alpha_1, alpha_2, alpha_3])
    print("M (cinemática directa):")
    print(M)

    # 3) Calcular inversa de M
    Minv = np.linalg.inv(M)
    print("\nM⁻¹ (cinemática inversa):")
    print(Minv)

    # 4) Ejemplo de uso: dada una velocidad deseada v = [vx, vy, ω]
    vx, vy, omega = 0.5, 0.2, 0.1  # m/s, m/s, rad/s
    v = np.array([vx, vy, omega])

    # 5) Obtener velocidades de ruedas s = [s1, s2, s3]
    s = Minv @ v
    print(f"\nEjemplo de uso:\n  v = [{vx}, {vy}, {omega}] → [s1, s2, s3] = {s}")

if __name__ == "__main__":
    main()
