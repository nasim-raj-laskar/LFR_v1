import os

project_structure = {
    "LFR-ESP32": [  
        "LFR-ESP32.ino",          
        "motor_control.h",
        "motor_control.cpp",
        "sensors.h",
        "sensors.cpp",
        "pid_controller.h",
        "pid_controller.cpp",
        "config.h",               
        "wifi_module.h",          
        "wifi_module.cpp"
    ],
    "test": {  
        "motor_test": ["motor_test.ino"],
        "sensor_test": ["sensor_test.ino"],
        "pid_test": ["pid_test.ino"],
        "wifi_test": ["wifi_test.ino"]  
    },
    "hardware": {
        "pcb": []  
    },
    "docs": {
        "images": [],
        "setup_guide.md": None
    },
    ".gitignore": None,
    "LICENSE": None,
    "README.md": None,
    "CONTRIBUTING.md": None
}

def create_structure(base_path, structure):
    for name, content in structure.items():
        path = os.path.join(base_path, name)

        if isinstance(content, dict):
            os.makedirs(path, exist_ok=True)
            create_structure(path, content)
        elif isinstance(content, list):
            os.makedirs(path, exist_ok=True)
            for file in content:
                open(os.path.join(path, file), 'w').close()
        elif content is None:
            open(path, 'w').close()

if __name__ == "__main__":
    create_structure(".", project_structure)
    print("✅ LFR-ESP32 project structure created successfully!")
