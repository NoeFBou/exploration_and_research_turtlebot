#!/usr/bin/env python3
import depthai as dai
import time

print("🔍 Test avec timeout augmenté...")

# Liste des devices
devices = dai.Device.getAllAvailableDevices()
print(f"✓ Devices trouvés: {len(devices)}")

if len(devices) == 0:
    print("❌ Aucun device trouvé")
    exit(1)

device_info = devices[0]
print(f"Device: {device_info.getMxId()}")
print(f"État: {device_info.state}")

# Pipeline minimaliste
pipeline = dai.Pipeline()
cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(256, 256)
xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

print("✓ Pipeline créé")

# Configuration USB avec timeout augmenté
config = dai.Device.Config()
config.version = dai.OpenVINO.Version.VERSION_2021_4

try:
    print("📷 Connexion avec timeout augmenté (30s)...")
    # Utiliser le device_info spécifique
    device = dai.Device(pipeline, device_info, usb2Mode=False)
    print("✓ Device initialisé !")
    
    q = device.getOutputQueue("rgb", maxSize=1, blocking=True)
    print("✓ Queue créée")
    
    print("⏳ Attente d'une frame...")
    frame = q.get()
    print(f"✓ Frame reçue : {frame.getCvFrame().shape}")
    
    device.close()
    print("✅ Test réussi !")
    
except Exception as e:
    print(f"❌ Erreur: {e}")
    import traceback
    traceback.print_exc()
