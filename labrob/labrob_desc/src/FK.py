import numpy as np
import sympy as sp


def create_transformation_matrix(xyz, rpy, theta, axis=[0, 0, 1]):
    """URDF verilerine göre homojen dönüş matrisi oluşturur."""
    x, y, z = xyz
    roll, pitch, yaw = rpy

    # Öteleme Matrisi (Translation)
    T_trans = sp.Matrix(
        [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]
    )

    # Sabit Oryantasyon Matrisi (RPY)
    cr, sr = sp.cos(roll), sp.sin(roll)
    cp, sp_p = sp.cos(pitch), sp.sin(pitch)
    cy, sy = sp.cos(yaw), sp.sin(yaw)

    R_x = sp.Matrix([[1, 0, 0, 0], [0, cr, -sr, 0], [0, sr, cr, 0], [0, 0, 0, 1]])

    R_y = sp.Matrix(
        [[cp, 0, sp_p, 0], [0, 1, 0, 0], [-sp_p, 0, cp, 0], [0, 0, 0, 1]]
    )

    R_z = sp.Matrix([[cy, -sy, 0, 0], [sy, cy, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    T_rpy = R_z * R_y * R_x

    # Motorun kendi dönüşü (Joint rotation - Genelde Z ekseninde)
    ct, st = sp.cos(theta), sp.sin(theta)
    if axis == [0, 0, 1]:
        T_joint = sp.Matrix(
            [[ct, -st, 0, 0], [st, ct, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        )
    elif axis == [0, 1, 0]:
        T_joint = sp.Matrix(
            [[ct, 0, st, 0], [0, 1, 0, 0], [-st, 0, ct, 0], [0, 0, 0, 1]]
        )
    else:
        T_joint = sp.eye(4)

    return T_trans * T_rpy * T_joint


def main():
    # Sembolik değişkenler (Motor Açıları)
    q1, q2, q3 = sp.symbols('q1 q2 q3')

    print("--- Pinion Gear Merkezli Matrisler Hesaplanıyor ---")

    # 1. World -> Base Link (Sabit)
    T_world_base = create_transformation_matrix([0, 0, 0], [0, 0, 0], 0)

    # 2. Base Link -> Shoulder (q1)
    T_base_shoulder = create_transformation_matrix([0, 0, 0.05744], [0, 0, 0], q1, axis=[0, 0, 1])

    # 3. Shoulder -> Upper Arm (q2)
    T_shoulder_upper = create_transformation_matrix([-0.01790, -0.00570, 0.06556], [-1.5708, -2.79253, 0], q2, axis=[0, 0, 1])

    # 4. Upper Arm -> Lower Arm (q3)
    T_upper_lower = create_transformation_matrix([0.170, 0.080, 0.0209], [0, -3.1415, -0.698132], q3, axis=[0, 0, 1])

    # 5. Lower Arm -> Pinion Gear (Yeni Uç Noktamız)
    # URDF: origin xyz="0.14408 0 0.0179" rpy="1.5708 1.5708 0", axis="0 1 0"
    # Not: Pinion Gear'ın kendi dönüşü (kıskaç açısı) konumu değiştirmediği için theta=0 aldık.
    T_lower_pinion = create_transformation_matrix([0.14408, 0, 0.0179], [1.5708, 1.5708, 0], 0, axis=[0, 1, 0])

    # Toplam Dönüşüm Matrisi (World'den Pinion Gear Merkezine)
    T_total = T_world_base * T_base_shoulder * T_shoulder_upper * T_upper_lower * T_lower_pinion

    # Pinion Gear koordinat denklemleri
    X_expr = T_total[0, 3]
    Y_expr = T_total[1, 3]
    Z_expr = T_total[2, 3]

    print("\n[INFO] Pinion Gear Modeli Başarıyla Çıkarıldı.")

    # --- RViz Test Değerlerin (q1=1.571, q2=0.0, q3=0.0) ---
    test_q1 = 0.785
    test_q2 = 1.571
    test_q3 = 1.571

    x_val = X_expr.subs({q1: test_q1, q2: test_q2, q3: test_q3}).evalf()
    y_val = Y_expr.subs({q1: test_q1, q2: test_q2, q3: test_q3}).evalf()
    z_val = Z_expr.subs({q1: test_q1, q2: test_q2, q3: test_q3}).evalf()

    print("\n--- Örnek Hesaplama Sonucu (Pinion Gear) ---")
    print(f"Giriş Açıları -> q1: {test_q1}, q2: {test_q2}, q3: {test_q3}")
    print(f"Pinion Konumu -> X: {x_val:.6f} m, Y: {y_val:.6f} m, Z: {z_val:.6f} m")


if __name__ == "__main__":
    main()