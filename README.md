# Procena pozicije SFP portova

Procena 3D pozicije SFP optičkih portova pomoću tri kamere montirane na robotu,
u simulaciji. UR5e robot u Isaac Sim-u se nasumično postavlja ispred mrežne
kartice; svaka prihvaćena poza se snima, automatski se označava na osnovu stabla
transformacija, i koristi se za treniranje YOLO26 detektora. U radu se detekcije
iz tri kamere triangulišu u pozicije u svetskom koordinatnom sistemu, zajedno sa
kovarijansom, i porede se sa stvarnim vrednostima.

**Izmereno na celom lancu: medijana greške pozicije 0,43 mm**, mAP50 detektora
0,919, oko 6 ms po frejmu. Videti [docs/training.md](docs/training.md) i
[docs/dataset.md](docs/dataset.md) za način na koji su dobijeni ti brojevi i šta
oni predstavljaju.

Zadatak potiče sa Intrinsic-ovog takmičenja
[AI for Industry Challenge](https://www.intrinsic.ai/events/ai-for-industry-challenge).

---

## Šta repozitorijum sadrži

| | |
|---|---|
| Kompletan izvorni kod: sempler, snimač, ekstraktor, označavanje, izvoz, 4 ROS 2 čvora | |
| **Istrenirani detektor** | `runs/sfp_yolo26s_p2/weights/best.pt` (20 MB) |
| Potpuna konfiguracija treniranja i zapis po epohama | `args.yaml`, `results.csv` |
| **USD scena** | `isaacsim/scene.usd` (9,1 MB) |
| Čitljiv prikaz scene | `docs/scene_contract.yaml` |

**Šta nije uključeno, i zašto:**

| | |
|---|---|
| Skup podataka (2757 slika) i bag fajlovi (9,9 GB) | regenerišu se kroz pipeline; preveliki za git |
| `yolo26s.pt`, `yolo26n.pt` | Ultralytics-ove težine, već javno dostupne na [ultralytics/assets](https://github.com/ultralytics/assets/releases) `v8.4.0` |

---

## Preduslovi

Projekat je vezan za konkretne verzije, a neslaganja se najčešće ne prijavljuju
kao jasna greška nego kao čudno ponašanje. [CLAUDE.md](CLAUDE.md) objašnjava
svaku od njih.

**Hardver**

- NVIDIA grafička kartica. Treniranje je rađeno na RTX 5090 (Blackwell,
  `sm_120`), što **zahteva torch build sa CUDA 12.8 ili novijom** — stariji
  buildovi neće raditi na toj kartici.
- Oko 32 GB VRAM-a za treniranje sa podrazumevanim podešavanjima; za inferencu
  je dovoljno oko 1 GB.
- Disk: jedno pokretanje od 1000 epizoda proizvodi oko 10 GB bag fajlova i oko
  324 MB ekstrahovanih podataka.

**Softver**

| | Verzija | Napomena |
|---|---|---|
| Ubuntu | 24.04 | |
| ROS 2 | **Jazzy** | Python 3.12 — to je bitno, videti ispod |
| Isaac Sim | **5.1.0** | potrebno samo za simulacioni deo |
| Python (sistemski) | 3.12 | dolazi uz ROS; pokreće ekstraktor i testove |
| torch | 2.9.1+cu130 | u virtuelnom okruženju, videti instalaciju |
| ultralytics | 8.4.155 | sadrži YOLO26 |
| OpenUSD | 25.11 | samo za regenerisanje prikaza scene |

**U igri su četiri Python interpretera i izbor pogrešnog je najčešći uzrok
problema.** `./run.sh` bira ispravan za svaki korak — koristite njega umesto
direktnog pozivanja skripti. Ukratko:

| Interpreter | Za šta |
|---|---|
| `~/isaacsim/python.sh` (3.11) | sempler, unutar Isaac Sim-a |
| `/usr/bin/python3` (3.12) | ekstraktor, označavanje, izvoz, testovi |
| `~/.venv/bin/python` (3.12) | sve što koristi torch — treniranje, detektor |
| OpenUSD venv | generisanje `scene_contract.yaml` |

Detektorski čvor se pokreće pod **venv** interpreterom, a ne sistemskim, i to
funkcioniše samo zato što su ROS 2 Jazzy i venv oba Python 3.12 — `cp312`
ekstenzije `rclpy` biblioteke se tada uredno učitavaju. Isaac Sim nosi Python
3.11 i nema tu mogućnost, zbog čega simulacija komunicira sa ROS-om preko
OmniGraph čvora umesto preko `rclpy`.

---

## Instalacija

```bash
git clone https://github.com/Cevilko/port_position_estimation.git
cd port_position_estimation
```

**1. ROS 2 Jazzy** — pratite
[zvanično uputstvo](https://docs.ros.org/en/jazzy/Installation.html), zatim:

```bash
sudo apt install ros-jazzy-vision-msgs ros-jazzy-rosbag2-storage-mcap
```

`vision_msgs` koriste detektor i triangulacija; mcap plugin je format u kom se
snimaju bag fajlovi.

**2. Virtuelno okruženje sa torch-om i ultralytics-om.** Sve što koristi torch
pokreće se odavde, nikada pod sistemskim interpreterom:

```bash
python3 -m venv ~/.venv
~/.venv/bin/pip install --index-url https://download.pytorch.org/whl/cu130 torch
~/.venv/bin/pip install ultralytics==8.4.155
```

Ako okruženje držite na drugom mestu, postavite `TRAIN_PYTHON=/putanja/do/python`.
Provera da li je kartica prepoznata:

```bash
~/.venv/bin/python -c "import torch; print(torch.cuda.get_device_name(0))"
```

**3. Build ROS 2 radnog prostora:**

```bash
./run.sh build
```

**4. Isaac Sim 5.1** — potreban samo za simulacioni deo. Instalirajte ga u
`~/isaacsim`, ili postavite `ISAAC_PYTHON` na njegov `python.sh`.

**Provera instalacije:**

```bash
./run.sh check      # 129 testova + svežina prikaza scene
```

---

## Korišćenje

### Pokretanje istreniranog detektora (bez Isaac Sim-a)

Model prepoznaje jednu klasu, `sfp_port`. Nad slikama ili direktorijumom:

```bash
~/.venv/bin/yolo detect predict \
    model=runs/sfp_yolo26s_p2/weights/best.pt \
    source=<slika-ili-direktorijum> imgsz=1152 save=True
```

**Uvek navedite `imgsz=1152`.** Portovi su oko 17 piksela u izvornoj rezoluciji;
podrazumevanih 640 ih svodi na oko 9 piksela i detektor deluje mnogo lošije nego
što jeste.

### Pokretanje lanca za percepciju u realnom vremenu

Četiri terminala, ili ih pokrenite u pozadini pa ih sve zaustavite sa
`./run.sh stop`. Svakom je potreban izvor `sensor_msgs/Image` poruka — Isaac
Sim, snimljeni bag, ili prava kamera.

```bash
./run.sh detect         # slike      -> vision_msgs/Detection2DArray
./run.sh triangulate    # detekcije  -> PoseWithCovarianceStamped, svetski sistem
./run.sh error          # procene    -> odstupanje od /tf, po portu
./run.sh rviz           # sirovi i označeni tokovi jedan pored drugog
```

Dve stvari koje će vas inače koštati popodneva:

- **`use_sim_time` mora da odgovara načinu na koji je scena pokrenuta.**
  Podrazumevano je `true`, što je ispravno kada scenu vodi `./run.sh sample`
  (tada se u toku rada dodaje `/clock` publisher). Ručno pokrenut Isaac Sim ili
  bag pušten bez `--clock` ne objavljuju `/clock`, pa čvor i dalje detektuje ali
  mu tajmeri nikada ne okinu — nema izveštaja o radu i, što je gore, nema
  upozorenja kada prestane da prima slike. U tom slučaju dodajte
  `-p use_sim_time:=false`.
- **Ništa pokrenuto u pozadini se ne gasi samo.** Samo detektor drži oko 1 GB
  VRAM-a neograničeno. `./run.sh stop` ih zaustavlja, a
  `./run.sh stop --dry-run` prvo ispisuje šta je pokrenuto.

### Generisanje skupa podataka (potreban Isaac Sim)

Dva terminala — **snimač mora da radi pre pokretanja semplera**, inače se poze
prihvataju a podaci se tiho ne snimaju.

```bash
# terminal 1
./run.sh recorder

# terminal 2
./run.sh sample --samples 1000 --headless     # oko 2 h; ~12 pokušaja po prihvaćenoj pozi
```

Zatim, nakon zaustavljanja snimača (da bi rosbag2 upisao `metadata.yaml`):

```bash
./run.sh extract                              # bag  -> rosbag_samples/
./run.sh yolo --drop-inconsistent             #      -> yolo_dataset/
./run.sh train pretrained=yolo26s.pt epochs=100
```

`--drop-inconsistent` je bitan: bez njega slika zadrži oznaku za jedan port, dok
drugi, vidljiv ali neoznačiv port ostaje tretiran kao pozadina.

### Sve komande

| Korak | Radnja |
|---|---|
| `build` | colcon build ROS 2 radnog prostora |
| `recorder` | servira `/record_rosbag` i upisuje mcap bag po pozivu |
| `sample` | nasumično postavlja poze u Isaac Sim-u i okida snimanje |
| `extract` | bag → slike, transformacije i parametri kamera na disk |
| `bbox` | projektuje portove u jedan frejm kao 2D okvire (`--annotate` da se iscrtaju) |
| `yolo` | ekstrahovani frejmovi → Ultralytics skup podataka |
| `train` | trenira detektor; podrazumevane vrednosti su podešene za objekte od ~17 px |
| `detect` | pokreće detektor nad živim temama kamera |
| `triangulate` | detekcije iz više kamera → 3D poza sa kovarijansom |
| `error` | poredi procene sa `/tf`, uparivanje jedan na jedan |
| `rviz` | otvara RViz sa podešenim rasporedom |
| `contract` | regeneriše `docs/scene_contract.yaml` iz scene |
| `check` | testovi + svežina prikaza scene — pokrenuti pre komitovanja |
| `stop` | zaustavlja sve što je ova skripta pokrenula |
| `info`, `topics` | pregled bag fajla / lista aktivnih tema |

`./run.sh` bez argumenata ispisuje istu listu sa punim objašnjenjima.

---

## Kako se delovi uklapaju

```
isaacsim/scene.usd                    UR5e + 3 kamere + ROS 2 OmniGraph
        │                             (binarno; videti docs/scene_contract.yaml)
        │
scripts/randomize_visible_joints.py   nasumično bira poze ruke i nosača dok oba
        │                             ulaza porta ne budu vidljiva centralnoj
        │                             kameri -- u vidnom polju, okrenuti ka njoj,
        │                             nezaklonjeni -- pa okida ROS 2 servis
        │  /record_rosbag  (std_srvs/Trigger)
        ▼
ros_ws/src/bag_recorder_node          pri svakom pozivu upisuje 3x(slika +
        │                             camera_info) i /tf u mcap bag
        ▼
rosbags/<ime>/                        snimljeni podaci (van git-a)
        │
scripts/extract_rosbag_samples.py     svaki snimljeni frejm: tri slike,
        ▼                             CameraInfo i transformacije kamera i
rosbag_samples/<ime>/                 portova, kao JPEG + YAML
        │
scripts/export_yolo_dataset.py        projektovani okviri -> Ultralytics skup
        ▼
yolo_dataset/ -> ./run.sh train -> runs/<ime>/weights/best.pt
        │
yolo_detector_node -> port_triangulator_node -> port_error_node
                                      detekcija, triangulacija, provera tačnosti
```

Nijedan granični okvir se ne crta ručno: oznake se projektuju iz snimljenih
transformacija i poznatih dimenzija otvora porta.

---

## Dokumentacija

| | |
|---|---|
| [CLAUDE.md](CLAUDE.md) | **pročitati pre pokretanja bilo čega** — verzije, interpreteri, redosled pokretanja i sve zamke pronađene do sada |
| [docs/pipeline.md](docs/pipeline.md) | ceo tok, korak po korak, sa matematikom |
| [docs/components.md](docs/components.md) | šta radi svaka skripta i svaki čvor |
| [docs/dataset.md](docs/dataset.md) | kako je napravljen skup podataka i koja su mu ograničenja |
| [docs/training.md](docs/training.md) | tok treniranja i šta rezultat zaista znači |
| [docs/scene_contract.yaml](docs/scene_contract.yaml) | **generisano** — šta scena objavljuje, u čitljivom obliku |

Dokumentacija je na engleskom jeziku.

---

## Objavljene ROS teme

Iz scene:

| Tema | Tip |
|---|---|
| `/{center,left,right}_camera/image` | `sensor_msgs/Image`, 1152×1024 `rgb8` |
| `/{center,left,right}_camera/camera_info` | `sensor_msgs/CameraInfo`, fx=fy=997,66, cx=576, cy=512 |
| `/tf` | `tf2_msgs/TFMessage` — zglobovi ruke, optički koordinatni sistemi kamera, oba ulaza porta |
| `/record_rosbag` | `std_srvs/Trigger` (servira `bag_recorder_node`) |

Iz čvorova za percepciju:

| Tema | Tip |
|---|---|
| `<camera>/detections` | `vision_msgs/Detection2DArray` |
| `<camera>/detections_image` | `sensor_msgs/Image` — označena slika, vidljiva u RViz-u |
| `/port_triangulator_node/port_<n>/pose` | `geometry_msgs/PoseWithCovarianceStamped`, svetski sistem |
| `/port_error_node/port_<n>/error` | `std_msgs/Float64` — odstupanje u metrima |

RViz nema prikaz za `Detection2DArray`, pa `rviz/dipl.rviz` prikazuje označene
slike umesto toga.

---

## Poznata ograničenja

- **Identitet porta je pozicioni, a ne semantički.** `port_0` iz triangulacije
  odgovara stvarnom `sfp_port_0_entrance` u otprilike polovini slučajeva.
  Razlikovanje bi zahtevalo treću referentnu tačku ili praćenje kroz vreme.
- **Odziv detektora ograničavaju bočne kamere**, koje sempler ne proverava na
  zaklonjenost: 0,997 na centralnoj kameri, 0,888 i 0,843 na bočnim. To je
  posledica načina označavanja, a ne slabosti modela.
- **Optički kabl je isključen u sceni** (`/UR5e_gripper/cable` je deaktiviran) i
  bio je isključen tokom celog rada, pa ga nema ni na jednom renderu, a provera
  zaklonjenosti nikada nije odbacila pozu koju bi on zaklonio.
- **Kovarijansa je namerno pesimistična** — podrazumevano `pixel_sigma=1.5`
  precenjuje nesigurnost oko 8 puta u odnosu na izmerenu grešku.
- **Svi brojevi potiču iz jedne prostorije, jednog rasporeda i jednog rendera**,
  uz validaciju iz istog pokretanja kao i treniranje. Oni pokazuju da je
  geometrija ispravna; ne govore ništa o pravoj kameri.
- Scena referencira modele apsolutnim putanjama ka `~/IsaacLab`, od kojih tri
  više ne postoje. U tom obliku nije prenosiva između računara.

---

## Licenca

Projekat je licenciran pod **GNU Affero General Public License v3.0**. Pun tekst
se nalazi u [LICENSE](LICENSE).

Koristi [Ultralytics](https://github.com/ultralytics/ultralytics) YOLO26, koji je
i sam pod AGPL-3.0. `yolo_detector_node` ga uvozi, a
`runs/sfp_yolo26s_p2/weights/best.pt` je treniran iz Ultralytics-ovih `yolo26s.pt`
težina, pa je taj model izvedeno delo — zbog čega se težine i kompletna
konfiguracija treniranja distribuiraju ovde zajedno sa izvornim kodom.

**Šta AGPL-3.0 znači za vas.** Smete da koristite, proučavate, menjate i dalje
distribuirate ovaj rad, uključujući i komercijalno, pod uslovom da izvedena dela
takođe objavite pod AGPL-3.0 sa kompletnim pripadajućim izvornim kodom. Član 13
proširuje tu obavezu i na mrežno korišćenje: ako pokrećete izmenjenu verziju i
korisnici joj pristupaju preko mreže, morate im ponuditi izvorni kod. Ako vam ti
uslovi ne odgovaraju, Ultralytics prodaje komercijalnu licencu koja uklanja AGPL
obavezu za njihov deo.

**Komponente trećih lica.** Tri fajla u
`ros_ws/src/bag_recorder_node/test/` su Copyright 2015 Open Source Robotics
Foundation i ostaju pod originalnom **Apache-2.0** licencom — Apache-2.0 je
kompatibilna sa AGPL-3.0 u ovom smeru, dok bi prelicenciranje tuđih fajlova bilo
neispravno. Ultralytics-ove težine `yolo26s.pt` i `yolo26n.pt` se ovde ne
distribuiraju; preuzmite ih sa
[ultralytics/assets](https://github.com/ultralytics/assets/releases).
