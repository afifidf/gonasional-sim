# Checklist pertandingan (OP3, stack `aroc_soccer`)

Bagian yang bisa dicek mesin dijalankan dengan `preflight`; sisanya dicek manual.

## Sebelum berangkat (sekali)

- [ ] Kalibrasi kamera C920 di **1280×720** (mode yang dipakai `usb_cam`), isi `fx fy cx cy`
      di `config/robot.yaml`. `camcalibrasi.yaml` lama adalah 640×480, jangan dipakai.
- [ ] Motion file robot berisi page **80** (walk ready), **15** (duduk), **142/143** (bangun),
      **155/156** (tendang kiri/kanan). Page tendang kosong di file bawaan ROBOTIS.
- [ ] Model YOLO dilatih ulang dengan foto lapangan asli + data simulasi, diekspor OpenVINO.
      Kelas: `bola, tiang_gawang, titik_penalti, titik_tengah, sudut_lapangan, robot`.
- [ ] `config/soccer.yaml` → `field`: ukuran lapangan, lebar gawang, jarak titik penalti
      sesuai aturan lomba tahun ini.
- [ ] Unit test lulus: `python3 -m pytest -q test/` di `src/GoNasional`.

## Di venue (sekali per lapangan)

- [ ] Ukur ulang lapangan; samakan dengan `field.*` (salah ukuran = lokalisasi meleset).
- [ ] Cek pencahayaan: jalankan `vision` dengan `show_window:=true`, pastikan bola, tiang,
      titik penalti terdeteksi dari 0.5–4 m. Kalau tidak, tambah foto venue ke dataset.
- [ ] WiFi 5 GHz tim: semua robot satu subnet; `team_comm.address` = broadcast subnet itu
      (mis. `192.168.1.255`). Port 10000 + nomor tim tidak diblokir.
- [ ] GameController: `team_number` sama dengan `teams.yaml` GameController.

## Tiap robot, sebelum tiap babak

- [ ] Baterai penuh (tegangan OpenCR > 11.5 V).
- [ ] `player_number` **berbeda** di tiap robot (`config/soccer.yaml` atau `-p player_number:=N`).
      Kalau sama, `team_comm` mencetak error "another robot also uses player_number".
- [ ] Jalankan stack:
      `ros2 launch op3_manager ...`, `ros2 run GoNasional vision ...`,
      `ros2 launch GoNasional aroc_soccer.launch.py`.
- [ ] Robot berdiri tegak, bola di depan, lalu:
      `ros2 run GoNasional preflight`
      Semua baris harus PASS atau WARN yang dimengerti (mis. GameController belum nyala).
- [ ] Lokalisasi TRACKING setelah kepala scan beberapa detik (field_viewer / preflight).
- [ ] Tes satu tendangan di pinggir lapangan.

## Saat bertanding

- Robot jatuh → `motion_manager` bangun otomatis (maks 3 kali), lalu berhenti dan menunggu
  tombol / perintah `stand`.
- GameController hilang di tengah babak → robot berhenti; tombol `start` untuk main manual.
- Setelah penalti dicabut, lokalisasi direset ke "masuk dari garis samping" (`reentry`).
