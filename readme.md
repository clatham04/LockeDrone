# Locke Drone

> An autonomous, controller-free drone capable of human recognition, self-stabilization, and intelligent follow tracking — all built on a $200 consumer drone platform.

---

## Description

**Locke Drone** is an autonomous flight project that transforms a standard consumer drone into a fully self-driving, human-following aerial system — no controller required.

Once tossed into the air by hand, the drone immediately activates its onboard stabilization system, orients itself, and begins tracking the nearest detected person. Using real-time human recognition, the drone intelligently maintains a safe following distance and adjusts its flight path automatically to keep pace with the subject.

**Key capabilities:**
- **Hand-launch activation** — simply toss the drone into the air and it takes over from there
- **Auto-stabilization** — instantly corrects orientation and achieves stable hover upon launch
- **Human recognition** — detects and identifies a person as the designated follow target
- **Autonomous following** — tracks the subject in real time without any remote input
- **Safe distance management** — maintains an approximate, collision-aware distance from the target at all times
- **Self-driving flight** — fully autonomous navigation with no controller dependency

---

## Prerequisites

Before setting up the project, ensure you have the following installed:

- Python 3.9+
- OpenCV (`cv2`)
- NumPy
- A compatible drone SDK (e.g., DJI Tello SDK, or MAVLink for custom builds)
- TensorFlow or PyTorch (for human recognition model)
- Git

---

## Project Layout

```
locke-drone/
│
├── src/                          # Core source code
│   ├── main.py                   # Entry point — launches drone system
│   ├── drone/
│   │   ├── controller.py         # Low-level drone communication & commands
│   │   ├── stabilizer.py         # Auto-stabilization logic on hand-launch
│   │   └── flight_manager.py     # High-level autonomous flight coordination
│   │
│   ├── vision/
│   │   ├── human_detector.py     # Human recognition model & inference
│   │   ├── tracker.py            # Real-time subject tracking logic
│   │   └── distance_estimator.py # Approximate distance calculation
│   │
│   └── utils/
│       ├── config.py             # Global configuration and constants
│       └── logger.py             # Logging utilities
│
├── models/                       # Pre-trained or custom ML model files
│   └── human_detection_model/
│
├── tests/                        # Unit and integration tests
│   ├── test_stabilizer.py
│   ├── test_tracker.py
│   └── test_controller.py
│
├── docs/                         # Documentation and diagrams
│   └── architecture.md
│
├── scripts/                      # Helper and setup scripts
│   └── setup_env.sh
│
├── requirements.txt              # Python dependencies
├── .env.example                  # Example environment variables
├── .gitignore
└── README.md
```

---

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/locke-drone.git
cd locke-drone
```

### 2. Create a Virtual Environment

```bash
python -m venv venv
source venv/bin/activate        # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure Environment Variables

```bash
cp .env.example .env
# Edit .env with your drone's IP address, port, and model paths
```

### 5. Connect Your Drone

Ensure your drone is powered on and your machine is connected to the drone's Wi-Fi network (or via USB depending on your setup). Refer to `docs/architecture.md` for drone-specific connection steps.

### 6. Run the System

```bash
python src/main.py
```

Once running, toss the drone gently upward — it will stabilize, detect you, and begin following automatically.

---

## Configuration

Key settings can be found in `src/utils/config.py`:

| Parameter | Description | Default |
|---|---|---|
| `FOLLOW_DISTANCE_MIN` | Minimum safe following distance (meters) | `1.5` |
| `FOLLOW_DISTANCE_MAX` | Maximum follow distance before catching up (meters) | `3.0` |
| `STABILIZE_DELAY_MS` | Time to allow stabilization after launch (ms) | `1500` |
| `DETECTION_CONFIDENCE` | Minimum confidence threshold for human detection | `0.75` |

---

## Contributing

Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

---

## License

[MIT](LICENSE)