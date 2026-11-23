# GoNasional — ROS 2 Robot OP3 Sepak Bola

Package ROS 2 (`ament_python`) untuk robot humanoid **ROBOTIS OP3**. Program utama
menjalankan state machine robot (`state`) dan estimasi jarak bola (`trigonocak`),
dilengkapi vision YOLO, kontrol kepala berbasis PID, dan GUI tuning PID.

Dikembangkan untuk ROS 2 Jazzy / Python 3.12.

Ada dua stack di package ini:

- **`aroc_soccer` (baru)**: lokalisasi, striker behavior tree, dan team play 3 robot.
  Diuji di simulasi Webots [op3-krsbi-sim](https://github.com/afifidf/op3-krsbi-sim).
- **`state` (lama)**: state machine follow-head tim. Dijelaskan di bagian bawah.

## Stack `aroc_soccer`

```
kamera ─ vision (YOLO) ──/vision/detections──┐
                                             ├─ perception_bridge ─ /perception/ball, /perception/landmarks
op3_manager ─ joint_states ─ robot_state_publisher (URDF, TF) ┘            │
IMU/yaw ───────────────────────────────── localization (MCL + Kalman bola) ─ /world_model
team_comm (UDP antar robot) ─ /team/state ─┐                               │
GameController ─ game_controller ─ /game_state ─ behavior (role + striker BT) ─ /cmd_vel, /motion/kick, /head/mode
                                                  motion_manager ─ /robotis/* (walking, action, kepala, bangun)
```

| Bagian | File | Isi |
|---|---|---|
| Persepsi | `core/ipm.py`, `core/landmarks.py`, `nodes/perception_bridge.py` | Piksel ke lantai lewat TF URDF (IPM). Landmark: tiang gawang (kaki tiang), titik penalti, titik tengah, sudut lapangan. Bola dipilih dengan cek geometri (ukuran kotak vs jarak, bukan di dalam kaki sendiri). Frame saat kepala/badan berputar cepat dibuang. |
| Lokalisasi | `core/mcl.py`, `core/ball_filter.py`, `nodes/localization_node.py` | Particle filter (300 partikel), heading dari IMU. Asosiasi satu-ke-satu per jenis landmark, gating chi-square saat TRACKING, state LOST dengan injeksi partikel acak. Bola: Kalman [x, y, vx, vy] di frame lapangan. |
| Behavior | `core/striker.py`, `core/team.py`, `nodes/behavior_node.py` | Cari bola, dekati dari belakang, orbit menghadap gawang, tendang. Team play: role kiper/striker/defender/supporter dibagi otomatis dan sama di semua robot; bola tim dipakai kalau robot tidak melihat bola. |
| Tim | `nodes/team_comm.py`, `core/team_packet.py` | Paket JSON lewat UDP broadcast (port 10000 + nomor tim), 3 Hz. Mendeteksi dua robot dengan `player_number` sama. |
| Gerak | `nodes/motion_manager.py`, `core/head_controller.py`, `core/fall_detector.py` | Satu-satunya node yang memerintah framework ROBOTIS: walking, tendang, bangun saat jatuh, scan kepala "berhenti lalu lihat". |
| Wasit | `core/gamecontroller.py`, `nodes/game_controller_node.py` | Protokol GameController RoboCup HSL; tanpa GameController, tombol `start` untuk main. |

Menjalankan di robot (setelah `op3_manager` dan kamera):

```bash
ros2 run GoNasional vision --ros-args -p model_path:=<folder_openvino> -p imgsz:=480 -p show_window:=false
ros2 launch GoNasional aroc_soccer.launch.py
ros2 run GoNasional preflight          # cek pra-pertandingan
```

Di simulasi: lihat README [op3-krsbi-sim](https://github.com/afifidf/op3-krsbi-sim).
`player_number` harus berbeda di tiap robot (`config/soccer.yaml`). Checklist lengkap:
[docs/checklist_pertandingan.md](src/GoNasional/docs/checklist_pertandingan.md).

Tes (tanpa ROS untuk logika inti, dengan ROS untuk konstruksi node):

```bash
cd src/GoNasional
python3 -m pytest -q test/test_core.py test/test_closed_loop.py test/test_ipm.py \
    test/test_landmarks.py test/test_team.py test/test_gamecontroller.py
python3 -m pytest -q test/test_nodes.py   # butuh workspace ROS yang sudah di-source
```

GitHub Actions menjalankan keduanya di tiap push.

## Stack lama (`state`)

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

- Path model YOLO di `main_vision.py` adalah parameter `model_path` (default
  `/home/aroc/BismilahNasional/src/GoNasional/model/nbaru_openvino_model`), dan
  `imgsz` harus sama dengan `metadata.yaml` model OpenVINO.
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
