# KURSAD40 — Cihaza Özel Kurulum Rehberi (SETUP.md)

> **Bu repoyu yeni klonladıysan buradan başla.** Bu dosya, KURSAD40 iş
> istasyonunu **senin cihazına** göre kurmak için gereken her adımı ve
> değiştirilmesi gereken **her cihaza-özel değeri** içerir.
>
> Bu doküman iki şekilde kullanılabilir:
> 1. **Yapay zekâ ile (önerilen):** §0'daki promptu bir AI asistanına
>    (Claude Code, Cursor, Copilot vb.) yapıştır. AI bu dosyayı okur,
>    §12'deki envanteri cihazında tek tek doğrular ve gereken
>    değişiklikleri yapar.
> 2. **Elle:** §1'den §11'e sırayla ilerle, §12'yi kontrol listesi olarak kullan.

**Referans cihaz (bu değerlerin alındığı sistem):** macOS 26.5.2 / Apple M4,
Homebrew Gazebo (gz-transport13 / gz-msgs10), PX4 SITL native build.
Ubuntu 22.04 + apt Gazebo yolu da her adımda ayrıca belirtilmiştir.

---

## 0. AI ile kurulum promptu (kopyala–yapıştır)

Repoyu klonladıktan sonra repo kökünde bir AI asistanı aç ve **aynen** şunu yaz:

```
Bu repoyu yeni klonladım. SETUP.md dosyasını oku ve bu sistemi BENİM cihazıma
göre kur.

Sırayla şunu yap:
1. Cihazımı tespit et: işletim sistemi, mimari (arm64/x86_64), Python sürümü,
   Gazebo sürümü ve Python binding yolu, Homebrew/apt prefix'i, kullanılabilir
   seri portlar, kamera cihazları.
2. SETUP.md §12'deki "Cihaza Özel Değer Envanteri" tablosundaki HER satırı
   tek tek gez. Her biri için: dosyadaki mevcut değeri oku, benim cihazımda
   doğru değerin ne olduğunu tespit et (tahmin etme — komut çalıştırarak
   doğrula), farklıysa değiştir, aynıysa "doğrulandı" diye işaretle.
3. Değiştiremediğin veya benden bilgi gerektiren değerler için (gerçek uçuş
   seri portu, servo AUX kanalı, saha HSV kalibrasyonu gibi) BANA SOR,
   varsayılan uydurma.
4. §1–§9'daki kurulum fazlarını uygula ve her fazın sonundaki doğrulama
   komutunu ÇALIŞTIR. Doğrulama geçmeden bir sonraki faza geçme.
5. §11'deki kendi GitHub remote'uma bağlanma adımlarını uygula.
6. Sonunda bana şunları raporla: değiştirilen her dosya ve satır, hangi
   değerden hangi değere, neden; doğrulaması geçen/kalan fazlar; ve benim
   karar vermem gereken açık maddeler.

Kural: SETUP.md §13'teki bilinen tuzakları okumadan hiçbir "çalışmıyor"
durumunu debug etme — oradaki hataların hepsi daha önce yaşandı ve nedeni
yazılı.
```

---

## 1. Ön koşullar

| Bileşen | macOS (Apple Silicon) | Ubuntu 22.04 |
|---|---|---|
| Toolchain | `brew install --cask px4-dev` veya PX4 dev setup | `Tools/setup/ubuntu.sh` |
| Gazebo | `brew install gz-harmonic` | `apt install gz-harmonic` |
| Python | 3.10–3.12 (Gazebo binding'i ile **aynı** minor sürüm) | sistem python3 |
| Git | submodule desteği ile | — |

**Kritik kısıt:** `gz.transport13` / `gz.msgs10` Python binding'leri sistem
Gazebo kurulumuna bağlıdır ve **pip ile venv'e kurulamaz**. Bu yüzden venv
`--system-site-packages` ile ya da `PYTHONPATH` enjeksiyonu ile kullanılır
(§3'e bak).

Python **minor** sürümü, binding'in derlendiği sürümle birebir eşleşmelidir:
derlenmiş uzantı `_transport.cpython-<XY>-darwin.so` şeklinde sürüme çapalıdır.
Homebrew aynı anda birden fazla Python sürümü için binding kurabildiğinden
(`python3.12/`, `python3.13/`, `python3.14/` site-packages'larının hepsinde bir
`gz/` dizini olabilir), yanlış olanı seçmek `No module named
'gz.transport13._transport'` hatası verir. Site-packages yolunu asla elle
yazma — çalışan interpreter'dan türet (§3 doğrulaması bunu yapar, §13.2'de
ayrıntısı var).

**Doğrulama:**
```bash
gz sim --version && python3 --version && git --version
```

---

## 2. Faz 1 — Klon ve submodule'ler

```bash
git clone --recursive https://github.com/artbymuusty/KURSAD40_WorkSpace.git
cd KURSAD40_WorkSpace
```

`--recursive` unutulduysa:
```bash
git submodule update --init --recursive
```

> **Not:** `Tools/simulation/gz` bu repoda **submodule değildir** — KURSAD40'a
> özel dünya/model dosyaları (`x500_mono_cam_down`, `blue_hexagon`,
> `red_triangle`, `PayloadDropSystem` eklentisi) doğrudan repo içinde
> versiyonlanır. Yukarı akış PX4'te bu bir submodule'dür; `.gitmodules`
> içindeki kayıt bilinçli olarak kaldırılmıştır. **Geri eklemeyin** — eklerseniz
> KURSAD40 modelleri yukarı akış modelleriyle ezilir.

**Doğrulama:**
```bash
test -f Tools/simulation/gz/models/x500_mono_cam_down/model.sdf && echo "KURSAD40 modelleri OK"
git submodule status | head -5
```

---

## 3. Faz 2 — Python ortamı

Launcher'lar interpreter'ı `.scripts/olds/v32/resolve_python.sh` üzerinden
**şu sırayla** arar:

1. `$VIRTUAL_ENV/bin/python` — aktif venv her zaman kazanır
2. `$KURSAD40_VENV/bin/python` — açık override
3. `~/Projects/kursad40-venv/bin/python` — kanonik macOS yolu
4. `<repo>/.scripts/.venv/bin/python` — orijinal Ubuntu repo-içi yolu
5. `python3` (PATH) — son çare

**Yani hiçbir dosyayı düzenlemene gerek yok** — sadece `KURSAD40_VENV`
değişkenini ayarla:

```bash
python3 -m venv --system-site-packages ~/Projects/kursad40-venv
export KURSAD40_VENV=$HOME/Projects/kursad40-venv     # kabuğun rc dosyasına ekle
"$KURSAD40_VENV/bin/pip" install -e .scripts/olds/v32/v32_flight_stack
```

Bağımlılıklar (`.scripts/olds/v32/v32_flight_stack/pyproject.toml`):
`mavsdk`, `opencv-python`, `numpy`, `pyyaml`, `pytest`, `pytest-asyncio`,
`ultralytics`, `pyzmq`, `psutil`.

**Doğrulama:**
```bash
source .scripts/olds/v32/resolve_python.sh && echo "PYTHON_BIN=$PYTHON_BIN"
"$PYTHON_BIN" -c "import mavsdk, cv2, zmq, psutil; print('deps OK')"
PYTHONPATH="$(brew --prefix 2>/dev/null || echo /usr)/lib/python$("$PYTHON_BIN" -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')/site-packages" \
  "$PYTHON_BIN" -c "from gz.transport13 import Node; print('gz bindings OK')"
```

Son komut hata verirse §13.2'ye bak.

---

## 4. Faz 3 — gz-transport kimliği (**en kritik cihaza-özel ayar**)

gz-transport'un **varsayılan partition'ı `<hostname>:<username>`'dir.** Bu
KURSAD40 için kabul edilemez: DHCP/ters-DNS ile türetilen hostname'ler ağ
değiştiğinde değişir, simülatör bir hostname altında başlar, sonra başlatılan
her süreç **farklı** bir partition hesaplar ve birbirlerini asla keşfedemezler.
Belirti: `gz topic -l` boş döner, `camera_service.py` "no live camera topic
found" der — Gazebo tüm bu süre boyunca 1280x960 @30Hz yayın yapıyorken.

Çözüm: sabit bir partition. Tek doğruluk kaynağı
[gz_env.sh](.scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh#L24-L25):

```bash
export GZ_PARTITION="${GZ_PARTITION:-kursad40}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
```

**Ne zaman değiştirmelisin:**

| Durum | Yapılacak |
|---|---|
| Tek geliştirici, tek makine | **Değiştirme.** `kursad40` kalsın. |
| Aynı ağda birden fazla kişi aynı anda sim çalıştırıyor | `GZ_PARTITION`'ı benzersiz yap: `export GZ_PARTITION=kursad40-<isim>` |
| Sim ile mission farklı makinelerde | `GZ_IP`'yi 127.0.0.1'den gerçek arayüz IP'sine çevir; **her iki makinede aynı** `GZ_PARTITION` |

**Değiştireceksen üç yeri birden değiştir** (aksi halde süreçler ayrışır):
- [gz_env.sh:24-25](.scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh#L24-L25)
- [gz_env.py:18-19](.scripts/olds/v32/v32_flight_stack/gz_system/gz_env.py#L18-L19)
- [process_manager.py:32-33](.scripts/olds/v32/process_manager.py#L32-L33)

En temizi hiçbirini elle düzenlememek ve kabuk ortamından override etmektir —
üçü de `setdefault`/`:-` kullanır, yani ortam değişkeni kazanır:
```bash
export GZ_PARTITION=kursad40-muusty
```

**Doğrulama:** simülatörü başlattıktan sonra (§5), *ayrı bir terminalde*:
```bash
source .scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh
gz topic -l | grep camera    # boş dönerse partition uyuşmuyor demektir
```

---

## 5. Faz 4 — PX4 SITL derleme ve simülasyonu başlatma

```bash
make px4_sitl_default          # ilk derleme; uzun sürer
./safe_sitl_launcher.sh        # simülatörü başlatan TEK doğru yol
```

`safe_sitl_launcher.sh` düz `make px4_sitl gz_x500_mono_cam_down`'dan farklı
olarak şunları da yapar (her biri yaşanmış bir hataya karşılık gelir):

1. `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION` ve `PYTHONPATH`'i temizler
2. `gz_env.sh`'yi source eder — sim ile Python araçları aynı partition'a düşsün
3. Yetim PX4/Gazebo süreçlerini **çapalanmış** pattern'lerle öldürür
   (`bin/px4$`, `gz sim`) — çapasız `px4` pattern'i çağıranın kendi kabuğunu
   öldürüyordu
4. Boş durumu doğrular, doğrulayamazsa FATAL verip çıkar
5. `make px4_sitl gz_x500_mono_cam_down` ile başlatır
6. Sonrasında takılı kalan LAND modunu `clear_land_mode.py` ile temizler

> **Make hedefi `gz_x500_mono_cam_down`'dır, `_payload` değildir.** Payload
> bırakma mekanizması (`PayloadDropSystem` eklentisi + `payload_blue`/
> `payload_red` link'leri) baz modelin içine konsolide edildi. Ayrı bir
> `_payload` varyantı **yoktur**; ona referans veren eski bir dosya bulursan
> yanlıştır.

**Doğrulama:**
```bash
source .scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh
gz topic -l | grep "/link/camera_link/sensor/camera/image"
```
Beklenen çıktı:
`/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`

---

## 6. Faz 5 — Simülasyon görevini çalıştır

Ayrı bir terminalde:
```bash
.scripts/olds/v32/run_mission_v32_gz.sh
```

Bu launcher `PYTHONPATH`'i `v32_flight_stack`'e ayarlar, `gz_env.sh`'yi source
eder, `resolve_python.sh` ile interpreter'ı bulur ve `gz_system/main_gz.py`'yi
çalıştırır.

**Simülasyon tarafında değişmesi gerekebilecek değerler:**

| Değer | Dosya | Varsayılan | Ne zaman değişir |
|---|---|---|---|
| MAVSDK bağlantısı | [gz_system.yaml:2](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L2) | `udp://:14540` | 14540 doluysa veya PX4 farklı port yayınlıyorsa |
| Kamera gz topic'i | [gz_system.yaml:9](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L9) | `/world/default/model/x500_mono_cam_down_0/...` | Dünya adı veya model instance adı değişirse |
| ZMQ kare kanalı | [gz_system.yaml:10](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L10) | `tcp://127.0.0.1:5555` | 5555 doluysa |
| Payload servisi | [gz_system.yaml:31](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L31) | `/v32/set_payload_state` | SDF eklenti servis adı değişirse |
| Araç model instance adı | [gz_payload_actuator.py:44](.scripts/olds/v32/v32_flight_stack/gz_system/gz_payload_actuator.py#L44) | `x500_mono_cam_down_0` | Farklı model/instance ile spawn edilirse |

> Kamera topic'inde `_0` soneki PX4'ün spawn ettiği **instance numarasıdır**.
> Birden fazla araç spawn edersen `_1`, `_2` olur. `camera_service.py` ve
> `process_manager.py` tam eşleşme bulamazsa `/link/camera_link/sensor/camera/image`
> **sonekine** göre otomatik keşif yapar, yani çoğu varyantı kendi bulur.

---

## 7. Faz 6 — Gerçek uçuş donanımı

`real_system.yaml` içindeki `TODO` işaretli her satır **senin donanımına**
göre doldurulmalıdır. Simülasyon değerlerini kopyalama.

### 7.1 Uçuş kontrolcüsü seri portu
[real_system.yaml:2](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L2) — varsayılan `serial:///dev/ttyUSB0:57600`

Portu bul:
```bash
# macOS
ls /dev/tty.usb* /dev/cu.usb*          # ör. /dev/tty.usbserial-0001
# Linux
ls /dev/ttyUSB* /dev/ttyACM*
dmesg | tail -20                        # kabloyu takıp çalıştır
```
Baud: telemetri radyosu tipik `57600`, doğrudan USB `921600`. Yanlış baud'da
MAVSDK sessizce bağlanamaz (timeout, hata değil).

Linux'ta izin gerekir:
```bash
sudo usermod -aG dialout $USER    # yeniden oturum aç
```

### 7.2 Kamera cihaz index'i
[real_system.yaml:4](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L4) — varsayılan `0`

macOS'ta `0` genelde dahili FaceTime kamerasıdır; **aşağı bakan kamera değildir.**
Index'i doğrula:
```bash
"$PYTHON_BIN" - <<'PY'
import cv2
for i in range(6):
    c = cv2.VideoCapture(i)
    if c.isOpened():
        ok, f = c.read()
        print(i, "OK", f.shape if ok else "kare yok")
        c.release()
PY
```

### 7.3 Servo / AUX kanalı
[real_system.yaml:14](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L14) — varsayılan `null`

`null` bırakılırsa payload bırakma **çalışmaz**. QGroundControl → Actuators
ekranından payload servosunun bağlı olduğu AUX çıkışını oku ve numarasını yaz.

### 7.4 Kontrol kazançları
[real_system.yaml:6-8](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L6-L8)

> **Bu değerleri simülasyondan kopyalama.** `gz_system.yaml` içinde
> `kp_vertical` 0.5'tir, `real_system.yaml`'da 0.3'tür ve bu **bilinçlidir**:
> gerçek araç kazancı bir simülatör ölçümü üzerine değiştirilemez. Fiziksel
> test ile kalibre et.

### 7.5 Payload kütlesi
[real_system.yaml:33-34](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L33-L34)

Gerçek payload 1.05 kg; simülasyon bilinçli olarak 0.15 kg taşır (ADR-011).
İki gerçek payload x500 gövdesine +%101 kütle demektir, bu da MPC_THR_HOVER'ı
ve itki/ağırlık oranını modelin tune'unun tamamen dışına çıkarır. Gerçek
gövden x500 **değil** — bu sayı ekibin kendi kütle/itki bütçesi içindir.

Çalıştır:
```bash
.scripts/olds/v32/run_mission_v32_real.sh
```

---

## 8. Faz 7 — Görüntü işleme kalibrasyonu

### 8.1 HSV eşikleri (saha ışığına özel)
[parameters.py:549-554](.scripts/olds/v32/v32_flight_stack/core/config/parameters.py#L549-L554)

```python
HSV_RED_LO_1  = (0, 40, 40)      HSV_RED_HI_1  = (15, 255, 255)
HSV_RED_LO_2  = (165, 40, 40)    HSV_RED_HI_2  = (180, 255, 255)
HSV_BLUE_LO   = (90, 80, 40)     HSV_BLUE_HI   = (140, 255, 255)
```

Kırmızı HSV'de 0/180'de sarıldığı için **iki aralık** kullanılır — birini
düzenleyip diğerini unutma. Bu değerler Gazebo'nun sentetik aydınlatmasına
göredir; **gerçek sahada güneş açısı, bulut ve zemin yansıması altında yeniden
kalibre edilmelidir.** Kalibrasyon için sahada kaydedilmiş kare üzerinde
`camera_viewer.py` ile bak.

### 8.2 Kamera intrinsics
[camera_intrinsics.py](.scripts/olds/v32/v32_flight_stack/core/detection/camera_intrinsics.py) —
**elle sabit girme.** FOV ve görüntü boyutu
`Tools/simulation/gz/models/mono_cam/model.sdf` dosyasından parse edilir; ikinci
bir doğruluk kaynağı bilinçli olarak yoktur. Gerçek kameraya geçerken SDF'yi
gerçek lensin FOV'una göre güncelle ya da açık konfigürasyon fallback'ini kullan.

### 8.3 YOLO modeli
Repodaki `.scripts/olds/v32/yolov8n.pt` **stok COCO-pretrained** modeldir ve
`MAVI_ALTIGEN` / `KIRMIZI_UCGEN` sınıflarını **içermez** — gerçek tespit
üretmez. Varsayılan tespit yolu HSV+kontur'dur. YOLO kullanacaksan yarışma
öncesi kendi sınıflarınla eğitilmiş bir ağırlık koy.

---

## 9. Faz 8 — Testler

```bash
export PYTHONPATH=$PWD/.scripts/olds/v32/v32_flight_stack
"$PYTHON_BIN" -m pytest .scripts/olds/v32/v32_flight_stack/tests -q
```

Testler donanım gerektirmez (mock backend kullanır). Kurulum sonrası ilk
yapman gereken doğrulama budur — geçmiyorsa Python ortamı eksiktir.

---

## 10. Mimari kararlar (ADR)

Bir davranışı değiştirmeden önce ilgili ADR'yi oku —
[docs/adr/](.scripts/olds/v32/v32_flight_stack/docs/adr/):

| ADR | Konu |
|---|---|
| ADR-004 | Mission Operations Center |
| ADR-005 | Dashboard migrasyonu |
| ADR-006 | macOS'ta dashboard boyama (Cocoa ana-thread kısıtı) |
| ADR-007 | Otonom görev başlatma |
| ADR-008 | Vision ömrü, telemetri kadansı, başlangıca dönüş |
| ADR-009 | Telemetri tazeliği, health backoff, merkezleme hızı |
| ADR-010 | Bırakma irtifası, gösterim sürekliliği, hareket overlay'i |
| ADR-011 | Simüle payload fiziği |

---

## 11. Kendi GitHub deponu bağlama

Bu repoyu kendi hesabına taşıyıp kendi değişikliklerini push'lamak için:

```bash
# 1. GitHub'da kendi deponu oluştur (boş, README'siz)
# 2. Remote'u kendi deponla değiştir:
git remote set-url origin https://github.com/<KULLANICI_ADIN>/<DEPO_ADIN>.git

# Alternatif: orijinali upstream olarak sakla
git remote rename origin upstream
git remote add origin https://github.com/<KULLANICI_ADIN>/<DEPO_ADIN>.git

# 3. Doğrula
git remote -v

# 4. Push
git push -u origin main
```

Kimlik doğrulama (HTTPS): GitHub artık şifre kabul etmez, **Personal Access
Token** gerekir — GitHub → Settings → Developer settings → Personal access
tokens → `repo` yetkisi. Ya da SSH kullan:
```bash
git remote set-url origin git@github.com:<KULLANICI_ADIN>/<DEPO_ADIN>.git
```

**Kendi cihaz ayarlarını commit'lemeden önce dikkat:** §12'deki değerlerin çoğu
cihaza özeldir. Kendi `GZ_PARTITION`'ını veya seri portunu commit'lersen
ekibin geri kalanının kurulumunu bozarsın. Tercihen:
- Değiştirmek yerine **ortam değişkeni** ile override et (`KURSAD40_VENV`,
  `GZ_PARTITION`, `GZ_IP` — hepsi override edilebilir)
- Gerçekten dosya değiştirmen gerekiyorsa ayrı bir branch'te tut

**Repoya commit'lenmeyenler** (`.gitignore`'da): `.scripts/olds/v32/logs/`
(çalışma anı görev log'ları, ~70 MB), `*.jsonl`, `*.pid`, `__pycache__/`,
`*.log`, venv dizinleri.

---

## 12. Cihaza Özel Değer Envanteri

**AI'ın tek tek doğrulaması gereken tam liste.** "Kaynak" sütunu değerin
nereden tespit edileceğini söyler — tahmin edilecek hiçbir satır yoktur.

### 12.1 Ortam / yol

| # | Değer | Yer | Varsayılan | Nasıl tespit edilir |
|---|---|---|---|---|
| E1 | Python interpreter | `KURSAD40_VENV` env | `~/Projects/kursad40-venv` | `source resolve_python.sh; echo $PYTHON_BIN` |
| E2 | Gazebo Python binding yolu | otomatik | macOS `$(brew --prefix)/lib/pythonX.Y/site-packages`, Linux `/usr/lib/python3/dist-packages` | `python3 -c "import gz.transport13"` |
| E3 | Homebrew prefix | `HOMEBREW_PREFIX` env | `/opt/homebrew` | `brew --prefix` (Intel Mac'te `/usr/local`) |
| E4 | Protobuf implementasyonu | otomatik `setdefault` | `python` | Değiştirme — C++ impl gz-transport ile çakışır |

### 12.2 gz-transport kimliği

| # | Değer | Yer | Varsayılan | Nasıl tespit edilir |
|---|---|---|---|---|
| G1 | `GZ_PARTITION` | [gz_env.sh:24](.scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh#L24), [gz_env.py:18](.scripts/olds/v32/v32_flight_stack/gz_system/gz_env.py#L18), [process_manager.py:32](.scripts/olds/v32/process_manager.py#L32) | `kursad40` | Tek kullanıcıysan değiştirme; ortak ağda benzersiz yap |
| G2 | `GZ_IP` | aynı üç dosya | `127.0.0.1` | Sim ve mission farklı makinedeyse gerçek arayüz IP'si: `ipconfig getifaddr en0` / `hostname -I` |

### 12.3 Simülasyon

| # | Değer | Yer | Varsayılan | Nasıl tespit edilir |
|---|---|---|---|---|
| S1 | MAVSDK bağlantısı | [gz_system.yaml:2](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L2) | `udp://:14540` | `lsof -i :14540` — doluysa değiştir |
| S2 | Kamera gz topic'i | [gz_system.yaml:9](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L9) | `.../x500_mono_cam_down_0/...` | `gz topic -l \| grep camera` |
| S3 | ZMQ adresi | [gz_system.yaml:10](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L10) | `tcp://127.0.0.1:5555` | `lsof -i :5555` |
| S4 | Payload servis adı | [gz_system.yaml:31](.scripts/olds/v32/v32_flight_stack/gz_system/config/gz_system.yaml#L31) | `/v32/set_payload_state` | `gz service -l \| grep payload` |
| S5 | Araç model instance adı | [gz_payload_actuator.py:44](.scripts/olds/v32/v32_flight_stack/gz_system/gz_payload_actuator.py#L44) | `x500_mono_cam_down_0` | `gz model -l` |
| S6 | SITL make hedefi | [safe_sitl_launcher.sh:114](safe_sitl_launcher.sh#L114) | `gz_x500_mono_cam_down` | Değiştirme (`_payload` varyantı yok) |
| S7 | QGC UDP portu | [parameters.py:176](.scripts/olds/v32/v32_flight_stack/core/config/parameters.py#L176) | `14550` | QGC varsayılanı |

### 12.4 Gerçek uçuş donanımı — **hepsi cihaza özel, hepsi doldurulmalı**

| # | Değer | Yer | Varsayılan | Nasıl tespit edilir |
|---|---|---|---|---|
| R1 | FC seri portu + baud | [real_system.yaml:2](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L2) | `serial:///dev/ttyUSB0:57600` **TODO** | `ls /dev/tty.usb*` (macOS) / `ls /dev/ttyACM*` (Linux) |
| R2 | Kamera index/pipeline | [real_system.yaml:4](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L4) | `0` **TODO** | §7.2'deki OpenCV tarama scripti |
| R3 | Servo/AUX kanalı | [real_system.yaml:14](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L14) | `null` **TODO** | QGroundControl → Actuators |
| R4 | `kp_horizontal` | [real_system.yaml:6](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L6) | `0.5` **TODO** | Fiziksel uçuş testi — simden kopyalama |
| R5 | `kp_vertical` | [real_system.yaml:7](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L7) | `0.3` **TODO** | Fiziksel uçuş testi (sim 0.5, kasıtlı fark) |
| R6 | `kp_altitude` | [real_system.yaml:8](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L8) | `0.5` **TODO** | Fiziksel uçuş testi |
| R7 | Gerçek payload kütlesi | [real_system.yaml:33](.scripts/olds/v32/v32_flight_stack/real_system/config/real_system.yaml#L33) | `1.05` kg | Payload'u tart |

### 12.5 Görüntü işleme — saha/lens'e özel

| # | Değer | Yer | Varsayılan | Nasıl tespit edilir |
|---|---|---|---|---|
| V1 | Kırmızı HSV (2 aralık) | [parameters.py:549-552](.scripts/olds/v32/v32_flight_stack/core/config/parameters.py#L549-L552) | `(0,40,40)-(15,255,255)` + `(165,40,40)-(180,255,255)` | Saha karesi üzerinde kalibrasyon |
| V2 | Mavi HSV | [parameters.py:553-554](.scripts/olds/v32/v32_flight_stack/core/config/parameters.py#L553-L554) | `(90,80,40)-(140,255,255)` | Saha karesi üzerinde kalibrasyon |
| V3 | Kamera FOV / çözünürlük | `Tools/simulation/gz/models/mono_cam/model.sdf` | SDF'den parse edilir | Gerçek lens FOV'u — kod içine sabit yazma |
| V4 | YOLO ağırlığı | `.scripts/olds/v32/yolov8n.pt` | stok COCO | Kendi sınıflarınla eğit; stok model tespit üretmez |

### 12.6 Görev parametreleri — yarışma şartnamesine bağlı, cihaza değil

Bunlar **cihaza özel değildir**; şartname veya tasarım kararıdır. Sadece
bilinçli bir gerekçeyle değiştir:
`MISSION_ALTITUDE_M=15.0`, `GOREV2_MAX_FLIGHT_DURATION_S=600` (şartname
zorunlu), `CENTERING_TOLERANCE_X/Y_NORM=0.01` (operatör kararı),
`NORMAL_MISSION_SPEED_M_S=None` **TODO — ekip dolduracak**,
`PAYLOAD_APPROACH_ALTITUDES_M=[10.0, 5.0, 0.45]`.

---

## 13. Bilinen tuzaklar — "çalışmıyor" debug'ına başlamadan önce oku

Aşağıdakilerin hepsi yaşandı, nedeni tespit edildi ve düzeltildi. Aynı
belirtiyi görürsen sıfırdan araştırma.

### 13.1 `gz topic -l` boş / "no live camera topic found" ama Gazebo yayında
**Neden:** gz-transport partition uyuşmazlığı. Varsayılan partition
`<hostname>:<username>` ve DHCP hostname'i ağla değişiyor.
**Çözüm:** Gazebo'ya konuşan **her** süreç `gz_env.sh`'yi source etmeli. §4.

### 13.2 `import gz.transport13` başarısız
İki ayrı nedeni var, hata mesajları farklıdır — ayırt et.

**(a) `ModuleNotFoundError: No module named 'gz'`**
Binding'ler sistem Gazebo'ya aittir, venv'e pip ile kurulamaz.
**Çözüm:** `PYTHONPATH`'e sistem site-packages'ı ekle — macOS'ta
`$HOMEBREW_PREFIX/lib/pythonX.Y/site-packages`, Linux'ta
`/usr/lib/python3/dist-packages`. `camera_service_manager.py` bunu otomatik
yapar; `camera_service.py`'yi doğrudan çalıştırırsan tam bu hatayı alırsın.

**(b) `ModuleNotFoundError: No module named 'gz.transport13._transport'`**
Bu daha sinsi: `gz` paketi **bulundu** ama içindeki derlenmiş uzantı
yüklenemedi. Nedeni **Python minor sürüm uyuşmazlığı** — `PYTHONPATH`'e
verdiğin site-packages, çalıştırdığın interpreter'dan farklı bir Python
sürümüne ait. Saf-Python `__init__.py` her sürümde aynıdır, o yüzden import
başlar; ama `.so` sürüme çapalıdır:

```
gz/transport13/_transport.cpython-312-darwin.so
gz/transport13/_transport.cpython-313-darwin.so
gz/transport13/_transport.cpython-314-darwin.so
```

Homebrew birden fazla Python sürümü için binding kurar, dolayısıyla
`python3.13/site-packages`'ı Python 3.14 ile kullanmak tam bu hatayı verir.

**Çözüm:** site-packages yolunu **elle yazma**, çalışan interpreter'dan türet:
```bash
PYVER=$("$PYTHON_BIN" -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")')
export PYTHONPATH="$(brew --prefix)/lib/python$PYVER/site-packages:$PYTHONPATH"
```
`process_manager.py` ve `camera_service_manager.py` zaten bunu yapar
(`sys.version_info` ile kurarlar) — bu yüzden onların üzerinden çalıştırmak
her zaman doğrudan çalıştırmaktan güvenlidir. İlgili site-packages dizininde
o sürüme ait `.so` hiç yoksa Gazebo'yu o Python sürümü için kur ya da
eşleşen sürümle bir venv oluştur.

### 13.3 protobuf çakışması
**Neden:** MAVSDK'nın protobuf stub'ları ile gz-transport13'ün protobuf
runtime'ı **aynı process'te birlikte var olamaz**.
**Çözüm:** `check_armable.py` gibi araçlar bilinçli olarak **ayrı subprocess**
olarak çalıştırılır, import edilmez. Bu ayrımı ortadan kaldırma.

### 13.4 "Launching Mission... " sonra sessiz donma
**Neden:** `main.py` çıktısı terminale değil `logs/mission.log`'a gidiyor;
Gazebo hazır değilken sıfır kare ile sonsuza kadar dönüyor.
**Çözüm:** `verify_gazebo_ready()` artık kamera topic'ini spawn öncesi
doğruluyor. Log'a bak, terminale değil.

### 13.5 `safe_sitl_launcher.sh` "FATAL" verip simülatörü hiç başlatmıyor
**Neden (düzeltildi):** Çapasız `pkill -9 -f "px4"` komut satırında "px4" geçen
**her** süreci öldürüyordu — çağıranın kendi yardımcı kabukları dahil — sonra
idle kontrolü aynı pattern'le tetikleniyordu.
**Çözüm:** Pattern'ler `bin/px4$` ve `gz sim`'e çapalandı. Ayrıca eski
`gz-sim` (tireli) pattern'i gerçek süreçle (`gz sim`, boşluklu) hiç eşleşmiyordu,
bu yüzden çökmüş bir dünya "temiz" sanılıp tekrar tekrar kullanılıyordu.

### 13.6 Araç arm olmuyor, tüm pre-arm kontrolleri geçiyor
**Neden:** İniş sonrası PX4 disarm ve ON_GROUND olsa bile `flight_mode=LAND`'de
kalıyor ve oradan arm etmeyi reddediyor (`is_armable=False`).
**Çözüm:** `clear_land_mode.py` HOLD komutu gönderiyor, ~2 saniyede temizliyor.
`safe_sitl_launcher.sh` bunu adım 5'te otomatik çalıştırır.

### 13.7 macOS'ta dashboard penceresi açılmıyor / cv2 exception
**Neden:** Cocoa **her** `cv2` GUI çağrısının ana thread'de olmasını şart koşar.
**Çözüm (ADR-006):** `main_gz.py` görev coroutine'ini worker thread'de
çalıştırır, ana thread `paint_bridge.py`'den kareleri çekip ~30 Hz `imshow`/
`waitKey` yapar. Linux/Windows değişmedi. Bu yapıyı bozma.

### 13.8 Launcher "No such file or directory" ile ölüyor
**Neden:** Eski launcher'lar `../../.venv/bin/python`'u doğrudan çağırıyordu —
Ubuntu repo-içi venv yolu, macOS'ta yok.
**Çözüm:** `resolve_python.sh`. §3.

---

## 14. Hızlı referans

```bash
# Simülasyon (iki terminal)
./safe_sitl_launcher.sh                        # terminal 1
.scripts/olds/v32/run_mission_v32_gz.sh        # terminal 2

# Gerçek uçuş
.scripts/olds/v32/run_mission_v32_real.sh

# Dual (gölge test: sim + gerçek eşzamanlı)
.scripts/olds/v32/run_mission_v32_dual.sh

# Sadece kamera görüntüsü
"$PYTHON_BIN" .scripts/olds/v32/v32_flight_stack/gz_system/camera_viewer.py

# Testler
PYTHONPATH=$PWD/.scripts/olds/v32/v32_flight_stack \
  "$PYTHON_BIN" -m pytest .scripts/olds/v32/v32_flight_stack/tests -q

# Ortam teşhisi
source .scripts/olds/v32/v32_flight_stack/gz_system/gz_env.sh && env | grep GZ_
source .scripts/olds/v32/resolve_python.sh && echo $PYTHON_BIN
gz topic -l | grep camera
```
