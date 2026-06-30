import numpy as np
import matplotlib.pyplot as plt

def rotation_matrix_z(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

def translation_matrix(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def rpy_to_matrix(r, p, y):
    Rx = np.array([[1, 0, 0, 0], [0, np.cos(r), -np.sin(r), 0], [0, np.sin(r), np.cos(r), 0], [0, 0, 0, 1]])
    Ry = np.array([[np.cos(p), 0, np.sin(p), 0], [0, 1, 0, 0], [-np.sin(p), 0, np.cos(p), 0], [0, 0, 0, 1]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0, 0], [np.sin(y), np.cos(y), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    return Rz @ Ry @ Rx

def forward_kinematics(theta1, theta2, theta3):
    T_world_base = np.eye(4)
    T_base_shoulder = translation_matrix(0, 0, 0.05744) @ rotation_matrix_z(theta1)
    T_shoulder_upper = translation_matrix(-0.0116, -0.0057, 0.06556) @ rpy_to_matrix(-1.5708, -1.5708, 0) @ rotation_matrix_z(theta2)
    T_upper_lower = translation_matrix(0.17, 0.08, 0.02183) @ rpy_to_matrix(-3.1416, 0, 1.5708) @ rotation_matrix_z(theta3)
    T_lower_gripper = translation_matrix(0.12408, 0, 0.01626) @ rpy_to_matrix(0, 1.5708, 1.5708)
    
    T_total = T_world_base @ T_base_shoulder @ T_shoulder_upper @ T_upper_lower @ T_lower_gripper
    return T_total[0, 3], T_total[1, 3], T_total[2, 3]

# --- MONTE CARLO SİMÜLASYONU VE METRİK HESAPLAMA ---
num_samples = 40000  
points = []

joint_min, joint_max = -1.570796, 1.570796

print("Ulaşılabilir bölge (Feasible Region) FK ile hesaplanıyor...")
for _ in range(num_samples):
    t1 = np.random.uniform(joint_min, joint_max)
    t2 = np.random.uniform(joint_min, joint_max)
    t3 = np.random.uniform(joint_min, joint_max)
    
    x, y, z = forward_kinematics(t1, t2, t3)
    points.append([x, y, z])

points = np.array(points)

# --- ANALİTİK SINIRLARIN BULUNMASI (IK İÇİN VERİ FİLTRELEME) ---
# Omuz merkezini (0, 0, 0.05744) referans alarak uç noktanın merkeze olan uzaklıklarını bulalım
shoulder_center = np.array([0, 0, 0.05744])
distances = np.linalg.norm(points - shoulder_center, axis=1)

max_reach = np.max(distances)
min_reach = np.min(distances)

print("\n--- RASCL Robot Kolu Geometrik Sınır Analizi ---")
print(f"Maksimum Uzanma Yarıçapı (Omuzdan): {max_reach:.4f} metre ({max_reach*100:.2f} cm)")
print(f"Minimum Uzanma Yarıçapı (Omuzdan): {min_reach:.4f} metre ({min_reach*100:.2f} cm)")
print(f"X Sınırları: [{points[:,0].min():.3f}, {points[:,0].max():.3f}] m")
print(f"Y Sınırları: [{points[:,1].min():.3f}, {points[:,1].max():.3f}] m")
print(f"Z Sınırları: [{points[:,2].min():.3f}, {points[:,2].max():.3f}] m")

# --- 3D GÖRSELLEŞTİRME VE IK SINIR ÇEMBERLERİ ---
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# Nokta bulutu
sc = ax.scatter(points[:,0], points[:,1], points[:,2], c=distances, cmap='plasma', s=0.3, alpha=0.5)

# Omuz merkezini işaretleyelim (Kinematik orijin)
ax.scatter([0], [0], [0.05744], color='red', s=50, label='Omuz Ekseni Merkez Noktası', zorder=10)

# Grafik Süslemeleri
ax.set_xlabel('X Ekseni (Metre)')
ax.set_ylabel('Y Ekseni (Metre)')
ax.set_zlabel('Z Ekseni (Metre)')
ax.set_title('RASCL Robot Kolu - Feasible Region ve Mesafe Haritası')
fig.colorbar(sc, label='Omuz Merkezine Olan Uzaklık (Metre)')
ax.legend()
ax.set_box_aspect([1,1,1]) 

plt.show()