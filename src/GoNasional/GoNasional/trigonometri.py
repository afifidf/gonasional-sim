import math

LH = 5.0   # cm - panjang offset kamera terhadap titik putar servo
H = 41.0   # cm - tinggi titik putar servo terhadap tanah

# Batas aman sudut servo (derajat), di luar ini geometri tidak valid
ALPHA_MIN_DEG = 1.0
ALPHA_MAX_DEG = 89.0


def hitung(servo_head_deg):
    """
    servo_head_deg : derajat (sudut servo head tilt / alpha),
                      diukur dari vertikal (0 = lurus ke bawah,
                      90 = horizontal)

    return:
        dict berisi jarak & tinggi dalam satuan cm,
        atau None jika sudut di luar rentang valid
        atau pembagian nol terdeteksi
    """

    # ==========================================
    # VALIDASI RANGE ALPHA
    # ==========================================
    # Di luar rentang ini, beta <= 0 atau >= 180,
    # sehingga tan(beta) berperilaku tidak wajar
    # (D negatif atau mendekati nol secara keliru).

    if not (ALPHA_MIN_DEG <= servo_head_deg <= ALPHA_MAX_DEG):
        return None

    servo_head = math.radians(servo_head_deg)

    # Persamaan 3.1 & 3.2
    x = LH * math.cos(servo_head)
    y = LH * math.sin(servo_head)

    # Persamaan 3.3
    H_top = H + y

    # Persamaan 3.4 -> beta = 180 derajat - (alpha + 90 derajat)
    beta_deg = 180.0 - (servo_head_deg + 90.0)

    # ==========================================
    # VALIDASI RANGE BETA
    # ==========================================
    # Jaga-jaga tambahan meski ALPHA_MIN/MAX sudah membatasi,
    # supaya perubahan konstanta di atas tidak diam-diam
    # menghasilkan beta invalid.

    if not (0.0 < beta_deg < 180.0):
        return None

    beta = math.radians(beta_deg)

    # ==========================================
    # GUARD PEMBAGIAN NOL / TAK HINGGA
    # ==========================================
    # tan(beta) mendekati 0 saat beta mendekati 0 derajat
    # atau 180 derajat (alpha mendekati 90 derajat atau -90 derajat)

    tan_beta = math.tan(beta)

    if abs(tan_beta) < 1e-6:
        return None

    # Persamaan 3.6
    D = H_top / tan_beta

    # Persamaan 3.8
    D_actual = D + x

    # ==========================================
    # VALIDASI HASIL AKHIR
    # ==========================================
    # Jarak fisik tidak boleh negatif; jika terjadi,
    # berarti ada kombinasi sudut/kalibrasi yang tidak
    # masuk akal secara geometris.

    if D_actual < 0:
        return None

    return {
        # "camera_x_cm": x,
        # "camera_y_cm": y,
        # "camera_height_cm": H_top,
        # "beta_deg": beta_deg,
        # "distance_camera_cm": D,
        "distance_robot_cm": D_actual,
    }


def dfov_to_hv_fov(dfov_deg: float, aspect_w: float, aspect_h: float):
    """Konversi diagonal FOV -> HFOV & VFOV berdasarkan aspect ratio."""

    d = math.sqrt(aspect_w ** 2 + aspect_h ** 2)
    dfov_rad = math.radians(dfov_deg)

    hfov_rad = 2 * math.atan((aspect_w / d) * math.tan(dfov_rad / 2))
    vfov_rad = 2 * math.atan((aspect_h / d) * math.tan(dfov_rad / 2))

    return math.degrees(hfov_rad), math.degrees(vfov_rad)


def get_offset_angle(bbox, image_width, image_height, hfov_deg, vfov_deg):
    """
    Hitung offset sudut (derajat) titik tengah bbox terhadap
    titik tengah frame (garis tengah horizontal & vertikal).

    Catatan: menggunakan pendekatan linear (deg per pixel).
    Untuk FOV lebar, pendekatan ini kurang akurat di tepi frame
    dibanding proyeksi perspektif (atan2 dengan fx/fy kamera).
    Gunakan konsisten dengan metode yang dipakai di trigonometri_node.py
    agar hasil jarak/sudut tidak bercampur antara dua pendekatan berbeda.

    bbox            : (x1, y1, x2, y2) dalam pixel
    return          : (offset_x_deg, offset_y_deg)
                      offset_x > 0 -> objek di kanan garis tengah
                      offset_x < 0 -> objek di kiri garis tengah
                      offset_y > 0 -> objek di bawah garis tengah
                      offset_y < 0 -> objek di atas garis tengah
    """

    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2
    cy = (y1 + y2) / 2

    deg_per_px_x = hfov_deg / image_width
    deg_per_px_y = vfov_deg / image_height

    offset_x_deg = (cx - image_width / 2) * deg_per_px_x
    offset_y_deg = (cy - image_height / 2) * deg_per_px_y

    return offset_x_deg, offset_y_deg