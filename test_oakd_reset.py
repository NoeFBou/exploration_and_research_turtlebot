#!/usr/bin/env python3
import depthai as dai
import time

print("🔍 Test avec reset du device...")

devices = dai.Device.getAllAvailableDevices()
print(f"Devices trouvés: {len(devices)}")

if len(devices) == 0:
    print("❌ Aucun device")
    exit(1)

device_info = devices[0]
print(f"Device: {device_info.getMxId()}")
print(f"État initial: {device_info.state}")

# Si le device est en état BOOTLOADER, on doit le booter d'abord
if device_info.state == dai.XLinkDeviceState.X_LINK_BOOTLOADER:
    print("⚠️  Device en mode BOOTLOADER - tentative de boot...")
    
# Essayer de créer un device SANS pipeline d'abord (juste pour booter)
try:
    print("📷 Étape 1: Boot du device...")
    device_boot = dai.Device()
    print("✓ Device booté")
    time.sleep(2)
    device_boot.close()
    print("✓ Device fermé")
    time.sleep(1)
    
except Exception as e:
    print(f"❌ Erreur boot: {e}")

# Maintenant essayer avec pipeline
try:
    print("📷 Étape 2: Chargement du pipeline...")
    pipeline = dai.Pipeline()
    cam = pipeline.create(dai.node.ColorCamera)
    cam.setPreviewSize(256, 256)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)
    
    device = dai.Device(pipeline)
    print("✅ Succès !")
    device.close()
    
except Exception as e:
    print(f"❌ Erreur pipeline: {e}")
    import traceback
    traceback.print_exc()
