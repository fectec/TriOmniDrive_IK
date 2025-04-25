import numpy as np

def matriz_cinematica_directa(alphas):
    """
    Construye la matriz M de cinemática directa (3×3) para un robot omnidireccional
    with tres ruedas cuyas orientaciones respecto al eje X del robot son alpha1, alpha2, alpha3.
    
    alphas: iterable de 3 valores en radianes [alpha1, alpha2, alpha3]
    devuelve: M (3×3 numpy.ndarray)
    """
    M = np.zeros((3, 3))
    for i, alpha in enumerate(alphas):
        # fila 0: cos(αᵢ + π/2)
        M[0, i] = np.cos(alpha + np.pi/2)
        # fila 1: sin(αᵢ + π/2)
        M[1, i] = np.sin(alpha + np.pi/2)
        # fila 2: 1 para la componente de rotación ω
        M[2, i] = 1.0
        #print(M)
    return M

def matriz_cinematica_inversa(alphas):
    """
    Calcula M⁻¹ a partir de los mismos ángulos.
    Si M no fuera invertible, puedes usar np.linalg.pinv en su lugar.
    """
    M = matriz_cinematica_directa(alphas)
    return np.linalg.inv(M)

if __name__ == "__main__":
    # Ejemplo con ángulos típicos: 30°, 150° y 270° (en radianes)
    alphas_deg = [30, 150, 270]
    alphas_rad = [np.deg2rad(a) for a in alphas_deg]

    M   = matriz_cinematica_directa(alphas_rad)
    Minv = matriz_cinematica_inversa(alphas_rad)

    #print("M (cinemática directa):\n", M)
    print("\nM⁻¹ (cinemática inversa):\n", Minv)
