#!/usr/bin/env python3
import numpy as np
from scipy.optimize import minimize

# =========================================================================
# 1. KUSURSUZ İLERİ KİNEMATİK (FK) MODELİ (enson.urdf BİREBİR PARAMETRELERİ)
# =========================================================================
def forward_kinematics_exact(q):
    """
    enson.urdf dosyasındaki karmaşık iç eksen dönüşümlerine göre
    pinion_gear merkezinin [X, Y, Z] konumunu hesaplar.
    """
    q1, q2, q3 = q

    def tf_matrix(xyz, rpy, theta, axis=[0,0,1]):
        T = np.eye(4)
        # RPY Dönüş matrisi matrisi (ROS Standart Z*Y*X)
        r, p, y = rpy
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        
        R_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        T[:3, :3] = R_z @ R_y @ R_x
        T[:3, 3] = xyz
        
        # Eklem kendi ekseni etrafında dönüşü
        T_j = np.eye(4)
        ct, st = np.cos(theta), np.sin(theta)
        if axis == [0,0,1]:
            T_j[:3, :3] = np.array([[ct, -st, 0], [st, ct, 0], [0, 0, 1]])
        elif axis == [0,1,0]:
            T_j[:3, :3] = np.array([[ct, 0, st], [0, 1, 0], [-st, 0, ct]])
        return T @ T_j

    # enson.urdf içindeki kinematik zincir ve origin değerleri
    T_w_b = np.eye(4)
    T_b_s = tf_matrix([0, 0, 0.057441], [0, 0, 0], q1)
    T_s_u = tf_matrix([-0.0116, -0.0057, 0.06556], [-1.5708, -1.5708, 0], q2)
    T_u_l = tf_matrix([0.17, 0.08, 0.02183], [-3.1416, 0, 1.5708], q3)
    T_l_g = tf_matrix([0.12408, 0, 0.01626], [0, 1.5708, 1.5708], 0.0) # gripper_base_joint
    T_g_p = tf_matrix([0, -0.0099, 0], [0, 1.57, 0], 0.0, axis=[0,1,0]) # pinion_joint

    T_total = T_w_b @ T_b_s @ T_s_u @ T_u_l @ T_l_g @ T_g_p
    return T_total[:3, 3]


# =========================================================================
# 2. SAYISAL TERS KİNEMATİK (NUMERICAL IK) ÇÖZÜCÜ
# =========================================================================
def inverse_kinematics_exact(target_x, target_y, target_z):
    """
    Hata minimizasyonu yöntemiyle hedef konuma götürecek 
    en pürüzsüz q1, q2, q3 açılarını bulur.
    """
    target_pos = np.array([target_x, target_y, target_z])
    
    # Maliyet Fonksiyonu (Hedef ile ulaşılan nokta arasındaki mesafe)
    def cost_function(q):
        current_pos = forward_kinematics_exact(q)
        return np.sum((target_pos - current_pos) ** 2)
    
    # Başlangıç tahmini (Robot dik durduğu için 0'lardan başlıyoruz)
    q_init = np.array([0.0, 0.0, 0.0])
    
    # enson.urdf dosyasındaki Kesin Eklem Limitleri
    bounds = [
        (-1.570796, 1.570796),  # shoulder_joint
        (-1.570796, 1.570796),  # upper_arm_joint
        (-1.570796, 1.570796)   # lower_arm_joint
    ]
    
    # Optimizasyon çözücü (SLSQP algoritması limitlere sadık kalır)
    result = minimize(cost_function, q_init, method='SLSQP', bounds=bounds, tol=1e-6)
    
    # Hata payı milimetrenin altındaysa başarıyla dön
    if result.fun < 1e-4:
        return result.x
    else:
        return None


# =============================================================================
# DOĞRULAMA VE HEDEF TEST ALANI
# =============================================================================
if __name__ == "__main__":
    print("==================================================")
    print("   TEMİZ URDF'E GÖRE KESİN TERS KİNEMATİK TESTİ   ")
    print("==================================================\n")

    # Patlayan o hedef koordinatını buraya veriyoruz:
    target_x = 0.2  
    target_y = 0.2  
    target_z = 0.4  

    print(f"Hedef Koordinat -> X: {target_x}, Y: {target_y}, Z: {target_z}")
    
    angles = inverse_kinematics_exact(target_x, target_y, target_z)
    
    if angles is not None:
        print("\n[BAŞARILI] Eksen Dönüşümlerine Uygun Açılar Bulundu:")
        print(f"-> shoulder_joint (q1)  : {angles[0]:.4f} rad")
        print(f"-> upper_arm_joint (q2) : {angles[1]:.4f} rad")
        print(f"-> lower_arm_joint (q3) : {angles[2]:.4f} rad")
        
        # Matematiksel Doğrulama (FK Kontrolü)
        check_pos = forward_kinematics_exact(angles)
        print(f"\n[DOĞRULAMA] Bu açılarla gidilen gerçek konum:")
        print(f"-> X: {check_pos[0]:.4f} m, Y: {check_pos[1]:.4f} m, Z: {check_pos[2]:.4f} m")
    else:
        print("\n[HATA] Bu konuma mevcut motor limitleriyle ulaşılması imkansız!")
