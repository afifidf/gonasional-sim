# GoNasional — ROS 2 Robot OP3 Sepak Bola

Package ROS 2 (`ament_python`) untuk robot humanoid **ROBOTIS OP3**. Program utama
menjalankan state machine robot (`state`) dan estimasi jarak bola (`trigonocak`),
dilengkapi vision YOLO, kontrol kepala berbasis PID, dan GUI tuning PID.

Dikembangkan untuk ROS 2 Jazzy / Python 3.12.

## Komponen Utama

| File | Node / Class | Fungsi |
|---|---|---|
| `main_state.py` | `RobotActionState` (`robot_state`) | State machine robot: pose naik/turun, scan, tracking bola, walking, rotate, kick, deteksi jatuh |
| `main_head_control.py` | `HeadControl` (class, dipakai `state`) | PID pan/tilt kepala, scan otomatis, orbit, terima retuning `/pid_params` |
| `sec_action.py` | `ActionPublisParam` | Publisher aksi: walking, rotate, kick, page action, fall recovery |
| `op3_orbit_walk.py` | `OP3OrbitWalker` | Orbit walking (yaw amplitude, durasi, periode) |
| `trigonorun.py` | `TrigonometriNode` (`trigonometri_node`) | Estimasi jarak bola dari bbox dan sudut head tilt |
| `trigonometri.py` | library | Perhitungan FOV, offset sudut, dan jarak |
| `main_trigono.py` | library | `DistanceEstimator` untuk jarak kamera |
| `main_vision.py` | `VisionYolo` (`usb_cam_yolo`) | YOLO OpenVINO: publish `/obj_detect` dan `/vision/jarak_kamera` |
| `tunning_PID.py` | GUI + node (`pid_tuner_gui_node`) | Slider Kp/Ki/Kd, grafik realtime, publish `/pid_params` |
| `main_PID.py` | library | `PIDControl` dengan anti-windup dan output clamp |
| `ema_filter.py` | library | Filter EMA untuk jarak dan posisi bola |
| `led.py` | library | Kontrol LED OpenCR |
| `errorgawang.py`, `record.py`, `giuiimu.py`, `yaw_filter.py`, `kalman_filter.py` | utility | Gawang, rekam data, GUI IMU, filter yaw/Kalman |

`motion_PID.py` bukan source Python (arsip PyInstaller). Jangan diedit.

## Kebutuhan

- ROS 2 Jazzy
- Message package OP3: `op3_walking_module_msgs`, `robotis_controller_msgs`
  (biasanya dari workspace ROBOTIS, mis. `~/RobotisSoccerYawNew`)
- Vision: `cv_bridge`, `opencv-python`, `ultralytics`
- GUI tuner: `python3-tk`, `python3-matplotlib`

## Build

```bash
cd ~/BismilahNasional
colcon build --symlink-install
source install/setup.bash
```

Source workspace message OP3 terlebih dahulu bila belum:

```bash
source ~/RobotisSoccerYawNew/install/setup.bash
```

Dengan `--symlink-install`, edit file `.py` atau launch file yang **sudah ada**
langsung berlaku setelah node di-restart. Rebuild hanya perlu jika mengubah
`setup.py` / `package.xml`, menambah entry point, atau menambah file baru.

## Menjalankan

Semua node inti (state + trigonometri) sekaligus:

```bash
ros2 launch GoNasional gonasional.launch.py
```

Terpisah:

```bash
ros2 run GoNasional state        # state machine + head control
ros2 run GoNasional trigonocak   # estimasi jarak bola
ros2 run GoNasional vision       # YOLO, butuh kamera dan model
ros2 run GoNasional tunning      # GUI PID tuner
```

`state` dan `trigonocak` aman dijalankan bersamaan karena nama node-nya berbeda.
Launch tidak menjalankan kamera (`usb_cam`), `vision`, atau `tunning`.

## Tuning PID Kepala

1. Jalankan `state` dan `tunning`.
2. Geser slider Kp/Ki/Kd pan/tilt. Perubahan dikirim ke `/pid_params` lewat
   tombol **Kirim ke Robot** atau otomatis saat nilai berubah.
3. `HeadControl._pid_params_callback` langsung memanggil `setConstant()` dan
   `Init()` pada PID pan/tilt.
4. Nilai disimpan otomatis ke `pid_config.json` dan dibaca lagi saat `state`
   dijalankan ulang.

Lokasi file:

- Mode source/symlink: `src/GoNasional/pid_config.json`
- Mode copy install: `install/.../site-packages/pid_config.json`

Format:

```json
{
  "pan":  { "kp": 0.7,  "ki": 0.0, "kd": 0.025 },
  "tilt": { "kp": 0.77, "ki": 0.0, "kd": 0.015 }
}
```

Jika file tidak ada atau rusak, node memakai `DEFAULT_PID` di
`main_head_control.py` dan tetap berjalan.

## Topik ROS

Produksi:

| Topik | Tipe | Node |
|---|---|---|
| `/obj_detect` | `String` (`cx,cy` per baris) | `vision` |
| `/vision/jarak_kamera` | `Float32` | `vision` |
| `/Hasil/Filter` | `String` | `vision` |
| `/vision/hasil_servo` | `Float32` | `trigonocak` |
| `/vision/trigono_result` | `String` | `trigonocak` |
| `/pid_params` | `String` (JSON) | `tunning` |
| `/robotis/head_control/set_joint_states` | `JointState` | `state` |
| `/communication/head_state` | `String` | `state` |
| `/communication/robot_state` | `String` | `state` (`sec_action`) |
| `/communication/head_command` | `String` | `state` (`sec_action`) |
| `/robotis/action/page_num` | `Int32` | `state` |
| `/robotis/enable_ctrl_module` | `String` | `state` |
| `/robotis/walking/command` | `String` | `state` |
| `/robotis/walking/set_params` | `WalkingParam` | `state` |
| `/robotis/walking/velocity` | `Twist` | `state` |
| `/landmarak/posisi` | `Int32` | `state` (`sec_action`) |

Konsumsi:

| Topik | Tipe | Node |
|---|---|---|
| `/usb_cam_node/image_raw` | `Image` | `vision` |
| `/robotis/goal_joint_states` | `JointState` | `vision`, `trigonocak`, `state`, `sec_action` |
| `/robotis/present_joint_states` | `JointState` | `tunning` |
| `/robotis/open_cr/button` | `String` | `state` |
| `/robotis/open_cr/yaw` | `Float64` | `state` |
| `/robotis/open_cr/imu` | `Imu` | `state` |
| `/obj_detect` | `String` | `state` |
| `/vision/jarak_kamera` | `Float32` | `state`, `sec_action` |
| `/vision/bola_cx_cy` | `Float32MultiArray` | `trigonocak` (publisher eksternal) |
| `/vision/bola_posisi` | `String` | `state` (opsional) |

## Tombol OpenCR

| Tombol | Aksi |
|---|---|
| `user` | Pose naik: `IDLE → STAND_MODULE → STAND_ACTION → STAND_HEAD → SCAN`. Pose turun: `LOWER_MODULE → LOWER_ACTION → IDLE` |
| `mode` | Toggle scan kepala dan simpan yaw referensi |
| `start` | Saat `SCAN`: mulai/stop walking follow-head |

Alur follow-head (`_follow_loop`, 0.65 s):

1. Bola hilang > 0.5 s → orbit stop.
2. Yaw error >= 20 derajat → rotate.
3. Jarak bola <= 25 cm → kick (page kiri/kanan).
4. Selain itu → `follow_head` mengikuti arah kepala.

Deteksi jatuh memakai akselerasi IMU sumbu X (threshold 9) dan action page
142 (jatuh depan) / 143 (jatuh belakang).

## Catatan

- Path model YOLO di `main_vision.py` masih hardcoded ke
  `/home/aroc/BismilahNasional/src/GoNasional/model/nbaru_openvino_model`.
  Sesuaikan dengan model yang tersedia, mis.
  `src/GoNasional/model/boladekat_openvino_model`.
- `HeadControl` bukan node mandiri. Entry point `headcontrol` belum bisa
  dijalankan karena `main()` di `main_head_control.py` masih dikomentari;
  gunakan `ros2 run GoNasional state`.
- Beberapa entry point di `setup.py` masih menunjuk file yang tidak ada di
  source saat ini (`bangun`, `jalan`, `goNas`, `button`, `cek`, `imu`, `imucsv`,
  `imucek`, `visionSec`, `deteksi`, `kirimData`). Hanya perintah yang
  didokumentasikan di atas yang aktif.
- Saat build muncul warning `SyntaxError: source code string cannot contain
  null bytes` dari `motion_PID.py`. Build tetap selesai; warning ini tidak
  memengaruhi program.
