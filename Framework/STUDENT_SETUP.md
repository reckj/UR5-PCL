# UR5e Framework — Student Setup

This guide gets the robot framework running on your laptop. You work against a
**simulated** UR5e on your own machine — no robot or internet needed once set up.

**You only install one thing: Docker Desktop.** Everything else (Python, the
robot libraries, the simulator) lives inside Docker, so there is nothing else to
install — no Homebrew, no Node, no Python.

Works on Apple Silicon Macs (M1–M4), Intel Macs, and Windows 10/11.

---

## Step 1 — Install Docker Desktop

1. Go to **https://www.docker.com/products/docker-desktop/** and download Docker
   Desktop for your computer:
   - **Mac:** pick **Apple Chip** (M1/M2/M3/M4) or **Intel Chip** — if unsure,
     click the Apple menu  → *About This Mac* and look at "Chip" / "Processor".
   - **Windows:** download the Windows installer.
2. Install it:
   - **Mac:** open the downloaded `.dmg`, drag **Docker** into **Applications**,
     then open it from Applications.
   - **Windows:** run the downloaded installer and accept the defaults (it sets
     up the required "WSL 2" component for you; you may need to restart).
3. **Launch Docker Desktop.** The first time, you can skip/close any sign-in
   prompt — an account is not required. Wait until the whale icon in the menu bar
   (Mac) or system tray (Windows) is **steady, not animating**. That means Docker
   is running. Leave Docker Desktop running whenever you use the framework.

---

## Step 2 — Get the framework folder

The project lives in the GitHub repository **reckj/UR5-PCL**, with the framework
in a **`Framework`** subfolder. The easiest way — with **no extra software** — is
to download it as a ZIP:

1. Open the repository: **https://github.com/reckj/UR5-PCL**
2. Click the green **`< > Code`** button, then **Download ZIP**.
3. **Unzip** the downloaded file. You'll get a folder named **`UR5-PCL-main`**
   that contains a **`Framework`** folder — that `Framework` folder is the one you
   work in.
4. Move **`UR5-PCL-main`** to a **simple, local location** (your home folder or
   Documents).

   ⚠️ **Do not** put it inside a cloud-synced folder (iCloud Drive, OneDrive,
   Dropbox, Google Drive). Docker does not work reliably there. A plain local
   folder like `~/UR5-PCL-main` (Mac) or `C:\Users\YourName\UR5-PCL-main`
   (Windows) is best.

> **Optional — if you already have Git** (more convenient for updates), clone
> instead of downloading:
> ```bash
> git clone https://github.com/reckj/UR5-PCL.git
> ```
> This creates a `UR5-PCL` folder with the same `Framework` subfolder inside.
> Most students don't need this; the ZIP download requires nothing extra.

---

## Step 3 — Open a terminal in the `Framework` folder

You'll type a few commands. Open a terminal **inside the `Framework` subfolder**
(it's inside `UR5-PCL-main`):

- **Mac:** open the **Terminal** app (Applications → Utilities → Terminal). Type
  `cd ` (with a space), then **drag the `Framework` folder** from Finder onto the
  Terminal window and press **Enter**.
- **Windows:** open the `Framework` folder in File Explorer, click the address
  bar, type `powershell`, and press **Enter**.

To check you're in the right place, type `ls` (Mac) or `dir` (Windows) and press
Enter — you should see `docker-compose.yml`, `examples`, `scripts`, and so on.

---

## Step 4 — Build it (one time)

```bash
docker compose build
```

The **first** build downloads and assembles everything and takes several minutes
(needs internet). After that it's cached and fast. You only rebuild when the
framework is updated.

---

## Step 5 — Start the simulator and the 3D viewer

```bash
docker compose up -d ursim viewer
```

This starts the simulated robot and a 3D viewer. Then:

1. **Open the simulator** in your browser: **http://localhost:6080/vnc.html** →
   click **Connect**. You'll see the robot's control screen (PolyScope). Press
   the power button (bottom-left), then **ON**, then **START** to switch the
   simulated robot on.
   - On Apple Silicon Macs the simulator is emulated and **slow to start** — give
     it a minute or two the first time.
2. **Open the 3D viewer** in your browser: **http://localhost:7001/static/** —
   you'll see the arm. Drag to orbit, scroll to zoom.

---

## Step 6 — Run an example

Open a shell inside the framework:

```bash
docker compose run --rm dev bash
```

Now you're "inside" the container. Try:

```bash
python scripts/check_connection.py --target sim     # reads the robot's state
python examples/grid_process.py                      # runs the first example
```

Watch the arm move in the 3D viewer (the browser tab from Step 5). Type `exit`
to leave the shell.

This is your working loop: edit files in the `examples/` folder with any editor
on your laptop (changes apply immediately — no rebuild), then run them from a
`docker compose run --rm dev bash` shell.

---

## Stopping

When you're done:

```bash
docker compose down
```

This stops the simulator and viewer. Your files are untouched. Next time, start
again from Step 5 (no rebuild needed).

---

## Updating the framework

When the instructor pushes a new version to GitHub:

- **If you downloaded the ZIP:** download it again (**`< > Code` → Download ZIP**),
  unzip, and replace your old folder (keep it in the same non-cloud location).
- **If you cloned with Git:** open a terminal in the folder and run `git pull`.

Then run `docker compose build` once more and continue as usual.

---

## Troubleshooting

**"Cannot connect to the Docker daemon" / a socket error**
Docker Desktop isn't running. Open it and wait for the whale icon to go steady,
then retry.

**The 3D viewer page won't load (`localhost:7001`)**
Make sure `docker compose up -d ursim viewer` ran without errors. On a Mac, if
port 7001 is taken, it's usually a second copy already running — `docker compose
down` and start again.

**`check_connection` can't connect to the sim**
Make sure you pressed **ON** then **START** in the simulator (Step 5) and that it
finished initializing (it's slow on Apple Silicon).

**The first `docker compose build` fails**
You need an internet connection for the first build (it downloads the base
image and packages). Check your connection and retry.

**The simulator is very slow (Apple Silicon)**
Expected — the simulator only exists as an Intel program and runs under
emulation on Apple Silicon. It's functional, just patient-making.

---

## (Lab only) Connecting to the real robot

You normally work in simulation. Driving the **real** arm happens only in the lab
and only when supervised. Briefly: connect your laptop to the robot's network,
then start the shell with the robot's IP, e.g.

```bash
docker compose run --rm --no-deps -e UR_HOST=192.168.1.11 dev bash
```

Your instructor will walk you through this — never run motion on the real arm
unsupervised.
